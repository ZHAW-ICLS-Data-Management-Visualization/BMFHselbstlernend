import numpy as np
import matplotlib.pyplot as plt

# Parameter für die Sinuskurven
A1 = 1.0  # Amplitude der ersten Kurve
f1 = 1.0  # Frequenz der ersten Kurve
phi1 = 0  # Phasenverschiebung der ersten Kurve

A2 = 0.8  # Amplitude der zweiten Kurve
f2 = 1.0  # Frequenz der zweiten Kurve
phi2 = np.pi / 4  # Phasenverschiebung der zweiten Kurve (zeitliche Versetzung)

# Zeitachse
t = np.linspace(0, 2 * np.pi, 1000)

# Sinuskurven berechnen
y1 = A1 * np.sin(2 * np.pi * f1 * t + phi1)
y2 = A2 * np.sin(2 * np.pi * f2 * t + phi2)

# Plot erstellen
plt.figure(figsize=(10, 6))
plt.plot(t, y1, label=f'Kurve 1: A={A1}, f={f1}, φ={phi1}', color='blue')
plt.plot(t, y2, label=f'Kurve 2: A={A2}, f={f2}, φ={phi2:.2f}', color='orange')

# Zeitliche Verschiebung markieren
shift_time = phi2 / (2 * np.pi * f2)
plt.axvline(x=shift_time, color='gray', linestyle='--', label=f'Zeitverschiebung ≈ {shift_time:.2f}s')

# Beschriftung und Legende
plt.title('Optimierung von Sinuskurvenparametern durch den Roboter')
plt.xlabel('Zeit (s)')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)

# Grafik anzeigen
plt.tight_layout()
plt.show()