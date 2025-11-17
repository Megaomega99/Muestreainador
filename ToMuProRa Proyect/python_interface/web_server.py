"""
Backend FastAPI para la interfaz web de control del filtro IIR
Proporciona endpoints REST y WebSocket para comunicación en tiempo real
"""

from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict
import json
import asyncio
import os
import sys

# Importar módulos del proyecto
from filter_calculator import FilterCalculator
from serial_communication import ArduinoController, list_available_ports
import config

# Inicializar FastAPI
app = FastAPI(title="Control de Filtro IIR - Arduino")

# Configurar CORS para desarrollo
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Instancias globales
filter_calc = FilterCalculator(fs=config.DEFAULT_SAMPLE_RATE)
arduino = ArduinoController(baudrate=config.SERIAL_BAUDRATE, timeout=config.SERIAL_TIMEOUT)

# WebSocket connections activas
active_connections: List[WebSocket] = []

# ===== MODELOS DE DATOS =====

class FilterParameters(BaseModel):
    center_freq: float
    bandwidth: float
    magnitude_threshold: int  # Ya convertido de normalizado a Q15 por el frontend
    phase_threshold: int  # Ya convertido de grados a Q15 por el frontend
    prediction_delay: int = 76

class ConnectionRequest(BaseModel):
    port: str

class CommandRequest(BaseModel):
    command: str

# ===== WEBSOCKET =====

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            # Mantener conexión activa
            await websocket.send_json({"type": "ping", "status": "alive"})
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        active_connections.remove(websocket)

@app.websocket("/ws/data")
async def websocket_data_stream(websocket: WebSocket):
    """WebSocket para streaming de datos de visualización en tiempo real"""
    await websocket.accept()
    print("Cliente conectado al stream de datos")

    try:
        while True:
            if arduino.connected:
                # Leer datos del Arduino
                data = arduino.read_visualization_data()
                if data:
                    # Enviar al cliente
                    await websocket.send_json({
                        "type": "data",
                        "timestamp": data['timestamp'],
                        "voltage": data['voltage'],
                        "filtered": data['filtered'],
                        "magnitude": data['magnitude'],
                        "phase_degrees": data['phase_degrees'],
                        "pulse_active": data['pulse_active']
                    })

            # Pequeña pausa para no saturar
            await asyncio.sleep(0.001)  # 1ms

    except Exception as e:
        print(f"WebSocket data stream error: {e}")
    finally:
        print("Cliente desconectado del stream de datos")

async def broadcast_message(message: dict):
    """Envía mensaje a todos los clientes conectados"""
    for connection in active_connections:
        try:
            await connection.send_json(message)
        except:
            pass

# ===== ENDPOINTS DE PUERTOS SERIALES =====

@app.get("/api/ports")
async def get_ports():
    """Obtiene la lista de puertos seriales disponibles"""
    try:
        ports = list_available_ports()
        return {
            "success": True,
            "ports": [{"device": device, "description": desc} for device, desc in ports]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/connect")
async def connect_arduino(request: ConnectionRequest):
    """Conecta al Arduino en el puerto especificado"""
    try:
        if arduino.connect(request.port):
            await broadcast_message({
                "type": "log",
                "message": f"Conectado a {request.port}",
                "level": "success"
            })
            return {
                "success": True,
                "message": f"Conectado a {request.port}",
                "connected": True
            }
        else:
            return {
                "success": False,
                "message": f"No se pudo conectar a {request.port}",
                "connected": False
            }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/disconnect")
async def disconnect_arduino():
    """Desconecta del Arduino"""
    try:
        arduino.disconnect()
        await broadcast_message({
            "type": "log",
            "message": "Desconectado del Arduino",
            "level": "info"
        })
        return {
            "success": True,
            "message": "Desconectado",
            "connected": False
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/connection-status")
async def get_connection_status():
    """Obtiene el estado de la conexión"""
    return {
        "connected": arduino.connected
    }

# ===== ENDPOINTS DE FILTRO =====

@app.post("/api/calculate-filter")
async def calculate_filter(params: FilterParameters):
    """Calcula los coeficientes del filtro IIR y Hilbert"""
    try:
        # Calcular Q automáticamente
        q = params.center_freq / params.bandwidth if params.bandwidth > 0 else 2.0

        await broadcast_message({
            "type": "log",
            "message": f"Calculando filtro IIR: {params.center_freq} Hz, BW={params.bandwidth} Hz, Q={q:.3f}",
            "level": "info"
        })

        # Calcular coeficientes IIR
        result = filter_calc.calculate_bandpass_coefficients(
            params.center_freq,
            params.bandwidth
        )

        # Calcular coeficientes de Hilbert
        hilbert_result = filter_calc.calculate_hilbert_coefficients(
            params.center_freq,
            15
        )

        # Calcular información de retrasos
        delay_info = filter_calc.estimate_total_delay(
            result['center_delay_samples']
        )

        await broadcast_message({
            "type": "log",
            "message": f"Filtro calculado exitosamente. Retraso recomendado: {delay_info['recommended_prediction_delay']} muestras",
            "level": "success"
        })

        return {
            "success": True,
            "iir": {
                "b0_q15": int(result['b0_q15']),
                "b1_q15": int(result['b1_q15']),
                "b2_q15": int(result['b2_q15']),
                "a1_q15": int(result['a1_q15']),
                "a2_q15": int(result['a2_q15']),
                "center_freq": result['center_freq'],
                "bandwidth": result['bandwidth'],
                "q_factor": result['q_factor']
            },
            "hilbert": {
                "coefficients_q15": hilbert_result['coefficients_q15'],
                "num_taps": hilbert_result['num_taps']
            },
            "delay": delay_info,
            "plot_data": {
                "frequency": result['frequency'].tolist(),
                "magnitude_db": result['magnitude_db'].tolist(),
                "phase_degrees": (result['phase'] * 180 / 3.14159265359).tolist()  # Convertir radianes a grados
            }
        }
    except Exception as e:
        await broadcast_message({
            "type": "log",
            "message": f"Error al calcular filtro: {str(e)}",
            "level": "error"
        })
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/send-to-arduino")
async def send_to_arduino(params: FilterParameters):
    """Envía los parámetros calculados al Arduino"""
    try:
        if not arduino.connected:
            return {
                "success": False,
                "message": "No hay conexión con el Arduino"
            }

        # Calcular coeficientes
        result = filter_calc.calculate_bandpass_coefficients(
            params.center_freq,
            params.bandwidth
        )

        hilbert_result = filter_calc.calculate_hilbert_coefficients(
            params.center_freq,
            15
        )

        # Preparar parámetros para envío
        arduino_params = {
            'b0_q15': int(result['b0_q15']),
            'b1_q15': int(result['b1_q15']),
            'b2_q15': int(result['b2_q15']),
            'a1_q15': int(result['a1_q15']),
            'a2_q15': int(result['a2_q15']),
            'magnitude_threshold': params.magnitude_threshold,
            'phase_threshold': params.phase_threshold,
            'prediction_delay': params.prediction_delay
        }

        await broadcast_message({
            "type": "log",
            "message": "Enviando parámetros al Arduino...",
            "level": "info"
        })

        # Enviar parámetros IIR y detección
        if not arduino.send_filter_coefficients(arduino_params):
            await broadcast_message({
                "type": "log",
                "message": "Error al enviar parámetros IIR",
                "level": "error"
            })
            return {
                "success": False,
                "message": "Error al enviar parámetros IIR"
            }

        await broadcast_message({
            "type": "log",
            "message": "Parámetros IIR enviados exitosamente",
            "level": "success"
        })

        # Enviar coeficientes de Hilbert
        if arduino.send_hilbert_coefficients(hilbert_result['coefficients_q15']):
            await broadcast_message({
                "type": "log",
                "message": "Coeficientes Hilbert enviados exitosamente",
                "level": "success"
            })
            return {
                "success": True,
                "message": "Todos los parámetros enviados al Arduino"
            }
        else:
            await broadcast_message({
                "type": "log",
                "message": "Error al enviar coeficientes Hilbert",
                "level": "warning"
            })
            return {
                "success": True,
                "message": "Parámetros IIR enviados, pero falló el envío de Hilbert"
            }

    except Exception as e:
        await broadcast_message({
            "type": "log",
            "message": f"Error: {str(e)}",
            "level": "error"
        })
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/reset-arduino")
async def reset_arduino():
    """Restaura los valores por defecto en el Arduino"""
    try:
        if not arduino.connected:
            return {
                "success": False,
                "message": "No hay conexión con el Arduino"
            }

        await broadcast_message({
            "type": "log",
            "message": "Restaurando valores por defecto en Arduino...",
            "level": "info"
        })

        if arduino.reset_to_defaults():
            await broadcast_message({
                "type": "log",
                "message": "Valores por defecto restaurados en Arduino",
                "level": "success"
            })
            return {
                "success": True,
                "message": "Valores por defecto restaurados"
            }
        else:
            await broadcast_message({
                "type": "log",
                "message": "Error al restaurar valores por defecto",
                "level": "error"
            })
            return {
                "success": False,
                "message": "Error al restaurar valores por defecto"
            }

    except Exception as e:
        await broadcast_message({
            "type": "log",
            "message": f"Error: {str(e)}",
            "level": "error"
        })
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/default-parameters")
async def get_default_parameters():
    """Obtiene los parámetros por defecto (21 Hz) en unidades físicas"""
    return {
        "center_freq": config.DEFAULT_CENTER_FREQ,
        "bandwidth": config.DEFAULT_BANDWIDTH,
        "magnitude_threshold": config.DEFAULT_MAGNITUDE_THRESHOLD_NORMALIZED,
        "phase_threshold": config.DEFAULT_PHASE_THRESHOLD_DEGREES,
        "prediction_delay": config.DEFAULT_PREDICTION_DELAY
    }

# ===== ARCHIVOS ESTÁTICOS =====

# Servir archivos estáticos (CSS, JS, imágenes)
static_dir = os.path.join(os.path.dirname(__file__), "static")
if not os.path.exists(static_dir):
    os.makedirs(static_dir)

app.mount("/static", StaticFiles(directory=static_dir), name="static")

@app.get("/")
async def read_root():
    """Sirve la página principal"""
    html_path = os.path.join(os.path.dirname(__file__), "templates", "index.html")
    if os.path.exists(html_path):
        return FileResponse(html_path)
    else:
        return HTMLResponse(content="<h1>Interfaz no encontrada. Ejecute el servidor después de crear los templates.</h1>")

# ===== PUNTO DE ENTRADA =====

if __name__ == "__main__":
    import uvicorn
    print("=" * 60)
    print("Control de Filtro IIR - Arduino")
    print("Megaomega engineering for well-being © 2025")
    print("=" * 60)
    print(f"\nIniciando servidor web en http://localhost:{config.SERVER_PORT}")
    print("Presione Ctrl+C para detener el servidor\n")

    uvicorn.run(app, host=config.SERVER_HOST, port=config.SERVER_PORT, log_level=config.LOG_LEVEL)
