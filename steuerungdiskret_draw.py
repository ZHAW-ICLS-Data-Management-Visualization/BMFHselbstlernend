import numpy as np
import matplotlib.pyplot as plt

# Diskrete Positionen für jeden Motor
motor1_positions = [0, 1, 0.5]
motor2_positions = [1, 0.5, 1]

# Anzahl der Iterationen
iterations = 3

# Wiederhole die Positionen für jede Iteration
motor1_sequence = motor1_positions * iterations
motor2_sequence = motor2_positions * iterations

# Zeitpunkte für die Schritte
time_steps = np.arange(len(motor1_sequence))

# Plot erstellen
plt.figure(figsize=(10, 6))
plt.step(time_steps, motor1_sequence, where='post', label='Motor 1', color='blue')
plt.step(time_steps, motor2_sequence, where='post', label='Motor 2', color='orange')

# Iterationsgrenzen markieren
for i in range(1, iterations):
    plt.axvline(x=i * len(motor1_positions), color='gray', linestyle='--', alpha=0.5)

# Beschriftung und Legende
plt.title('Diskrete Bewegungen der Motoren über drei Iterationen')
plt.xlabel('Zeit (Schritte)')
plt.ylabel('Position')
plt.legend()
plt.grid(True)

# Grafik anzeigen
plt.tight_layout()
plt.show()
