"""
Interfaz gráfica para control de parámetros del filtro IIR
y comunicación con Arduino
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import threading
import serial.tools.list_ports
import os
from PIL import Image, ImageTk

from filter_calculator import FilterCalculator
from serial_communication import ArduinoController


class FilterControlGUI:
    """Interfaz gráfica principal para control del sistema"""

    def __init__(self, root):
        self.root = root
        self.root.title("Control de Filtro IIR - Arduino")
        self.root.geometry("1200x800")

        # Inicializar calculador y controlador
        self.filter_calc = FilterCalculator(fs=2000.0)
        self.arduino = ArduinoController()

        # Variables de control
        self.center_freq = tk.DoubleVar(value=21.0)
        self.bandwidth = tk.DoubleVar(value=9.0)
        # q_factor se calcula automáticamente: Q = fc / BW
        self.magnitude_threshold = tk.IntVar(value=50)
        self.phase_threshold = tk.IntVar(value=2730)
        self.prediction_delay = tk.IntVar(value=76)

        # Estado de conexión
        self.connected = False
        self.port_var = tk.StringVar()

        # Inicializar console como None (se creará en create_widgets)
        self.console = None
        self.current_filter = None

        # Crear interfaz
        self.create_widgets()
        self.update_port_list()

        # Cargar filtro por defecto después de crear la interfaz
        self.calculate_filter()

    def create_widgets(self):
        """Crea todos los widgets de la interfaz"""

        # Configurar peso de filas y columnas de la raíz
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=0)  # Header
        self.root.rowconfigure(1, weight=1)  # Contenido principal

        # ===== HEADER CON LOGO =====
        header_frame = ttk.Frame(self.root)
        header_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=10, pady=5)
        header_frame.columnconfigure(0, weight=1)

        # Cargar y mostrar logo en la parte derecha
        try:
            logo_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                     "with_padding.png")
            logo_image = Image.open(logo_path)

            # Redimensionar logo manteniendo proporción
            max_height = 60
            ratio = max_height / logo_image.height
            new_width = int(logo_image.width * ratio)
            logo_image = logo_image.resize((new_width, max_height), Image.Resampling.LANCZOS)

            self.logo_photo = ImageTk.PhotoImage(logo_image)
            logo_label = ttk.Label(header_frame, image=self.logo_photo)
            logo_label.grid(row=0, column=1, sticky=tk.E, padx=5)

            # Título a la izquierda
            title_label = ttk.Label(header_frame, text="Control de Filtro IIR - Arduino",
                                   font=('Arial', 12, 'bold'))
            title_label.grid(row=0, column=0, sticky=tk.W, padx=5)
        except Exception as e:
            print(f"No se pudo cargar el logo: {e}")
            # Si falla, solo mostrar título centrado
            title_label = ttk.Label(header_frame, text="Control de Filtro IIR - Arduino",
                                   font=('Arial', 12, 'bold'))
            title_label.grid(row=0, column=0, columnspan=2, pady=5)

        # Frame principal dividido en izquierda y derecha
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configurar peso de filas y columnas
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # ===== PANEL IZQUIERDO: CONTROLES =====
        left_frame = ttk.Frame(main_frame, padding="5")
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Sección de conexión serial
        self.create_serial_section(left_frame)

        # Sección de parámetros del filtro
        self.create_filter_section(left_frame)

        # Sección de parámetros de detección
        self.create_detection_section(left_frame)

        # Sección de información de retrasos
        self.create_delay_section(left_frame)

        # Botones de acción
        self.create_action_buttons(left_frame)

        # ===== PANEL DERECHO: VISUALIZACIÓN =====
        right_frame = ttk.Frame(main_frame, padding="5")
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Gráfica de respuesta en frecuencia
        self.create_frequency_plot(right_frame)

        # Consola de mensajes
        self.create_console(right_frame)

    def create_serial_section(self, parent):
        """Crea la sección de conexión serial"""
        frame = ttk.LabelFrame(parent, text="Conexión Serial", padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)

        ttk.Label(frame, text="Puerto:").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var,
                                       state="readonly", width=20)
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Button(frame, text="Actualizar", command=self.update_port_list).grid(
            row=0, column=2, padx=2)

        self.connect_btn = ttk.Button(frame, text="Conectar",
                                      command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=2)

        self.status_label = ttk.Label(frame, text="● Desconectado",
                                     foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=4, pady=5)

    def create_filter_section(self, parent):
        """Crea la sección de parámetros del filtro"""
        frame = ttk.LabelFrame(parent, text="Parámetros del Filtro IIR",
                              padding="10")
        frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)

        # Frecuencia central
        ttk.Label(frame, text="Frecuencia Central (Hz):").grid(
            row=0, column=0, sticky=tk.W, pady=3)
        freq_spinbox = ttk.Spinbox(frame, from_=1.0, to=500.0,
                                   textvariable=self.center_freq,
                                   width=10, increment=0.5)
        freq_spinbox.grid(row=0, column=1, sticky=tk.W, padx=5)

        # Ancho de banda
        ttk.Label(frame, text="Ancho de Banda (Hz):").grid(
            row=1, column=0, sticky=tk.W, pady=3)
        bw_spinbox = ttk.Spinbox(frame, from_=1.0, to=100.0,
                                textvariable=self.bandwidth,
                                width=10, increment=0.5)
        bw_spinbox.grid(row=1, column=1, sticky=tk.W, padx=5)

        # Etiqueta informativa de Q (solo lectura)
        ttk.Label(frame, text="Factor Q (calculado):").grid(
            row=2, column=0, sticky=tk.W, pady=3)
        self.q_label = ttk.Label(frame, text="2.33", foreground="blue")
        self.q_label.grid(row=2, column=1, sticky=tk.W, padx=5)

        # Botón de valores por defecto
        ttk.Button(frame, text="Valores por Defecto (21 Hz)",
                  command=self.load_default_values).grid(
            row=3, column=0, columnspan=2, pady=10)

    def create_detection_section(self, parent):
        """Crea la sección de parámetros de detección"""
        frame = ttk.LabelFrame(parent, text="Parámetros de Detección",
                              padding="10")
        frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)

        # Umbral de magnitud
        ttk.Label(frame, text="Umbral de Magnitud:").grid(
            row=0, column=0, sticky=tk.W, pady=3)
        mag_spinbox = ttk.Spinbox(frame, from_=1, to=1000,
                                 textvariable=self.magnitude_threshold,
                                 width=10, increment=1)
        mag_spinbox.grid(row=0, column=1, sticky=tk.W, padx=5)

        # Umbral de fase
        ttk.Label(frame, text="Umbral de Fase:").grid(
            row=1, column=0, sticky=tk.W, pady=3)
        phase_spinbox = ttk.Spinbox(frame, from_=100, to=10000,
                                   textvariable=self.phase_threshold,
                                   width=10, increment=10)
        phase_spinbox.grid(row=1, column=1, sticky=tk.W, padx=5)

        # Retraso de predicción
        ttk.Label(frame, text="Retraso de Predicción:").grid(
            row=2, column=0, sticky=tk.W, pady=3)
        delay_spinbox = ttk.Spinbox(frame, from_=0, to=500,
                                   textvariable=self.prediction_delay,
                                   width=10, increment=1)
        delay_spinbox.grid(row=2, column=1, sticky=tk.W, padx=5)
        ttk.Label(frame, text="muestras").grid(row=2, column=2, sticky=tk.W)

    def create_delay_section(self, parent):
        """Crea la sección de información de retrasos"""
        frame = ttk.LabelFrame(parent, text="Información de Retrasos",
                              padding="10")
        frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)

        self.delay_text = tk.Text(frame, height=8, width=40, state='disabled',
                                 font=('Courier', 9))
        self.delay_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL,
                                 command=self.delay_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.delay_text['yscrollcommand'] = scrollbar.set

    def create_action_buttons(self, parent):
        """Crea los botones de acción"""
        frame = ttk.Frame(parent, padding="10")
        frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=10)

        ttk.Button(frame, text="Calcular Coeficientes",
                  command=self.calculate_filter).grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W+tk.E)

        ttk.Button(frame, text="Enviar a Arduino",
                  command=self.send_to_arduino).grid(
            row=1, column=0, padx=5, pady=5, sticky=tk.W+tk.E)

        ttk.Button(frame, text="Restaurar Valores en Arduino",
                  command=self.reset_arduino_to_defaults).grid(
            row=2, column=0, padx=5, pady=5, sticky=tk.W+tk.E)

        ttk.Button(frame, text="Actualizar Gráfica",
                  command=self.update_plot).grid(
            row=3, column=0, padx=5, pady=5, sticky=tk.W+tk.E)

        frame.columnconfigure(0, weight=1)

    def create_frequency_plot(self, parent):
        """Crea la gráfica de respuesta en frecuencia"""
        frame = ttk.LabelFrame(parent, text="Respuesta en Frecuencia",
                              padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        parent.rowconfigure(0, weight=3)

        # Crear figura de matplotlib
        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Gráfica inicial
        self.update_plot()

    def create_console(self, parent):
        """Crea la consola de mensajes"""
        frame = ttk.LabelFrame(parent, text="Consola de Mensajes",
                              padding="10")
        frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        parent.rowconfigure(1, weight=1)

        self.console = scrolledtext.ScrolledText(frame, height=10,
                                                state='disabled',
                                                font=('Courier', 9))
        self.console.pack(fill=tk.BOTH, expand=True)

    # ===== MÉTODOS DE CONTROL =====

    def update_port_list(self):
        """Actualiza la lista de puertos seriales disponibles"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)

    def toggle_connection(self):
        """Conecta o desconecta del Arduino"""
        if not self.connected:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Seleccione un puerto serial")
                return

            if self.arduino.connect(port):
                self.connected = True
                self.connect_btn.config(text="Desconectar")
                self.status_label.config(text="● Conectado", foreground="green")
                self.log_message(f"Conectado a {port}")
            else:
                messagebox.showerror("Error", f"No se pudo conectar a {port}")
        else:
            self.arduino.disconnect()
            self.connected = False
            self.connect_btn.config(text="Conectar")
            self.status_label.config(text="● Desconectado", foreground="red")
            self.log_message("Desconectado")

    def load_default_values(self):
        """Carga los valores por defecto (21 Hz)"""
        self.center_freq.set(21.0)
        self.bandwidth.set(9.0)
        self.magnitude_threshold.set(50)
        self.phase_threshold.set(2730)
        self.prediction_delay.set(76)
        self.log_message("Valores por defecto cargados (21 Hz, BW=9 Hz)")
        self.calculate_filter()

    def calculate_filter(self):
        """Calcula los coeficientes del filtro"""
        try:
            freq = self.center_freq.get()
            bw = self.bandwidth.get()

            # Calcular Q automáticamente desde BW
            q = freq / bw if bw > 0 else 2.0

            # Actualizar el label de Q
            if hasattr(self, 'q_label'):
                self.q_label.config(text=f"{q:.3f}")

            self.log_message(f"Calculando filtro IIR: {freq} Hz, BW={bw} Hz, Q={q:.3f}")

            # Calcular coeficientes IIR (pasamos bw, q se calcula internamente)
            result = self.filter_calc.calculate_bandpass_coefficients(freq, bw)

            # Calcular coeficientes de Hilbert optimizados para esta frecuencia
            self.log_message(f"Calculando filtro Hilbert para {freq} Hz...")
            hilbert_result = self.filter_calc.calculate_hilbert_coefficients(freq, 15)

            # Guardar resultados
            self.current_filter = result
            self.current_hilbert = hilbert_result

            # Mostrar información de retrasos
            delay_info = self.filter_calc.estimate_total_delay(
                result['center_delay_samples'])

            self.display_delay_info(delay_info)

            # Actualizar gráfica
            self.update_plot()

            # Mostrar coeficientes IIR
            self.log_message(f"Coeficientes IIR Q15:")
            self.log_message(f"  b0_q15 = {result['b0_q15']}")
            self.log_message(f"  b1_q15 = {result['b1_q15']}")
            self.log_message(f"  b2_q15 = {result['b2_q15']}")
            self.log_message(f"  a1_q15 = {result['a1_q15']}")
            self.log_message(f"  a2_q15 = {result['a2_q15']}")

            # Mostrar coeficientes Hilbert
            self.log_message(f"Coeficientes Hilbert Q15:")
            hilbert_str = ', '.join([str(c) for c in hilbert_result['coefficients_q15']])
            self.log_message(f"  [{hilbert_str}]")

            self.log_message(f"Retraso recomendado: {delay_info['recommended_prediction_delay']} muestras")

        except Exception as e:
            messagebox.showerror("Error", f"Error al calcular filtro: {str(e)}")
            self.log_message(f"ERROR: {str(e)}")

    def display_delay_info(self, delay_info):
        """Muestra información de retrasos en el widget de texto"""
        # Si el widget de texto no existe todavía, simplemente imprimir
        if not hasattr(self, 'delay_text') or self.delay_text is None:
            print(f"Retraso total: {delay_info['total_delay_samples']:.2f} muestras")
            return

        self.delay_text.config(state='normal')
        self.delay_text.delete(1.0, tk.END)

        info = f"""Retrasos del Sistema:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Filtro IIR:    {delay_info['iir_delay_samples']:.2f} muestras
Filtro Hilbert: {delay_info['hilbert_delay_samples']:.2f} muestras
─────────────────────────────────
TOTAL:         {delay_info['total_delay_samples']:.2f} muestras
               {delay_info['total_delay_ms']:.2f} ms

Retraso Recomendado:
  {delay_info['recommended_prediction_delay']} muestras

Retraso Actual:
  {delay_info['current_prediction_delay']} muestras
"""
        self.delay_text.insert(1.0, info)
        self.delay_text.config(state='disabled')

    def update_plot(self):
        """Actualiza la gráfica de respuesta en frecuencia"""
        # Si no hay filtro calculado, mostrar gráfica vacía
        if self.current_filter is None:
            self.ax.clear()
            self.ax.set_xlabel('Frecuencia (Hz)')
            self.ax.set_ylabel('Magnitud (dB)')
            self.ax.set_title('Respuesta en Frecuencia del Filtro IIR')
            self.ax.grid(True, alpha=0.3)
            self.ax.text(0.5, 0.5, 'Calcula un filtro para ver la gráfica',
                        ha='center', va='center', transform=self.ax.transAxes,
                        fontsize=12, color='gray')
            self.canvas.draw()
            return

        result = self.current_filter

        self.ax.clear()
        self.ax.plot(result['frequency'], result['magnitude_db'],
                    'b-', linewidth=2)
        self.ax.axvline(result['center_freq'], color='r', linestyle='--',
                       label=f"Centro: {result['center_freq']} Hz")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('Frecuencia (Hz)')
        self.ax.set_ylabel('Magnitud (dB)')
        self.ax.set_title('Respuesta en Frecuencia del Filtro IIR')
        self.ax.legend()
        self.ax.set_xlim([0, 100])

        self.canvas.draw()

    def send_to_arduino(self):
        """Envía los parámetros al Arduino"""
        if not self.connected:
            messagebox.showwarning("Advertencia",
                                 "Debe conectarse al Arduino primero")
            return

        if not hasattr(self, 'current_filter'):
            messagebox.showwarning("Advertencia",
                                 "Debe calcular el filtro primero")
            return

        try:
            result = self.current_filter

            # Preparar parámetros IIR y detección
            params = {
                'b0_q15': result['b0_q15'],
                'b1_q15': result['b1_q15'],
                'b2_q15': result['b2_q15'],
                'a1_q15': result['a1_q15'],
                'a2_q15': result['a2_q15'],
                'magnitude_threshold': self.magnitude_threshold.get(),
                'phase_threshold': self.phase_threshold.get(),
                'prediction_delay': self.prediction_delay.get()
            }

            self.log_message("Enviando parámetros al Arduino...")

            # Enviar parámetros IIR y detección
            if not self.arduino.send_filter_coefficients(params):
                self.log_message("✗ Error al enviar parámetros IIR")
                messagebox.showerror("Error", "No se pudieron enviar los parámetros IIR")
                return

            self.log_message("✓ Parámetros IIR enviados exitosamente")

            # Enviar coeficientes de Hilbert si están disponibles
            if hasattr(self, 'current_hilbert'):
                hilbert_coeffs = self.current_hilbert['coefficients_q15']
                self.log_message("Enviando coeficientes Hilbert...")

                if self.arduino.send_hilbert_coefficients(hilbert_coeffs):
                    self.log_message("✓ Coeficientes Hilbert enviados exitosamente")
                    messagebox.showinfo("Éxito", "Todos los parámetros enviados al Arduino")
                else:
                    self.log_message("✗ Error al enviar coeficientes Hilbert")
                    messagebox.showwarning("Advertencia",
                                         "Parámetros IIR enviados, pero falló el envío de Hilbert")
            else:
                self.log_message("⚠ No hay coeficientes Hilbert calculados")
                messagebox.showinfo("Éxito", "Parámetros IIR enviados (Hilbert sin cambios)")

        except Exception as e:
            messagebox.showerror("Error", f"Error al enviar parámetros: {str(e)}")
            self.log_message(f"ERROR: {str(e)}")

    def reset_arduino_to_defaults(self):
        """Restaura los valores por defecto en el Arduino"""
        if not self.connected:
            messagebox.showwarning("Advertencia",
                                 "Debe conectarse al Arduino primero")
            return

        # Confirmar acción
        if not messagebox.askyesno("Confirmar",
                                   "¿Desea restaurar los valores por defecto (21 Hz) en el Arduino?"):
            return

        try:
            self.log_message("Restaurando valores por defecto en Arduino...")

            if self.arduino.reset_to_defaults():
                self.log_message("✓ Valores por defecto restaurados en Arduino")
                messagebox.showinfo("Éxito", "Valores por defecto restaurados en Arduino")
            else:
                self.log_message("✗ Error al restaurar valores por defecto")
                messagebox.showerror("Error", "No se pudieron restaurar los valores por defecto")

        except Exception as e:
            messagebox.showerror("Error", f"Error al restaurar valores: {str(e)}")
            self.log_message(f"ERROR: {str(e)}")

    def log_message(self, message):
        """Agrega un mensaje a la consola"""
        # Si la consola aún no existe, simplemente imprimir en stdout
        if self.console is None:
            print(message)
            return

        self.console.config(state='normal')
        self.console.insert(tk.END, f"{message}\n")
        self.console.see(tk.END)
        self.console.config(state='disabled')


def main():
    root = tk.Tk()
    app = FilterControlGUI(root)
    root.mainloop()

def send_to_arduino(self):
        """Envía los parámetros al Arduino"""
        if not self.connected:
            messagebox.showwarning("Advertencia",
                                 "Debe conectarse al Arduino primero")
            return

        if not hasattr(self, 'current_filter'):
            messagebox.showwarning("Advertencia",
                                 "Debe calcular el filtro primero")
            return

        try:
            result = self.current_filter

            # Preparar parámetros IIR y detección
            params = {
                'b0_q15': result['b0_q15'],
                'b1_q15': result['b1_q15'],
                'b2_q15': result['b2_q15'],
                'a1_q15': result['a1_q15'],
                'a2_q15': result['a2_q15'],
                'magnitude_threshold': self.magnitude_threshold.get(),
                'phase_threshold': self.phase_threshold.get(),
                'prediction_delay': self.prediction_delay.get()
            }

            self.log_message("Enviando parámetros al Arduino...")

            # Enviar parámetros IIR y detección
            if not self.arduino.send_filter_coefficients(params):
                self.log_message("✗ Error al enviar parámetros IIR")
                messagebox.showerror("Error", "No se pudieron enviar los parámetros IIR")
                return

            self.log_message("✓ Parámetros IIR enviados exitosamente")

            # Enviar coeficientes de Hilbert si están disponibles
            if hasattr(self, 'current_hilbert'):
                hilbert_coeffs = self.current_hilbert['coefficients_q15']
                self.log_message("Enviando coeficientes Hilbert...")

                if self.arduino.send_hilbert_coefficients(hilbert_coeffs):
                    self.log_message("✓ Coeficientes Hilbert enviados exitosamente")
                    messagebox.showinfo("Éxito", "Todos los parámetros enviados al Arduino")
                else:
                    self.log_message("✗ Error al enviar coeficientes Hilbert")
                    messagebox.showwarning("Advertencia",
                                         "Parámetros IIR enviados, pero falló el envío de Hilbert")
            else:
                self.log_message("⚠ No hay coeficientes Hilbert calculados")
                messagebox.showinfo("Éxito", "Parámetros IIR enviados (Hilbert sin cambios)")

        except Exception as e:
            messagebox.showerror("Error", f"Error al enviar parámetros: {str(e)}")
            self.log_message(f"ERROR: {str(e)}")

if __name__ == "__main__":
    print("Iniciando Interfaz Gráfica de Control...")
    print(" Megaomega engineering for well-being © 2025")
    main()
