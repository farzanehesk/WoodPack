import matplotlib.pyplot as plt

# Widths from the user-provided data
row1 = [
    0.100805, 0.163666, 0.118473, 0.276863, 0.092685, 0.133072, 0.143554, 0.173693, 0.122027,
    0.090866, 0.171363, 0.175537, 0.099172, 0.119859, 0.081984, 0.085309, 0.089675, 0.113789,
    0.093389, 0.103846, 0.097548, 0.116455, 0.092726, 0.094301, 0.102108, 0.100259, 0.124674,
    0.118665, 0.127225, 0.096301
]

row2 = [
    0.053507, 0.166985, 0.111324, 0.124601, 0.144516, 0.168777, 0.080394, 0.082391, 0.091018,
    0.118842, 0.077081, 0.089808, 0.104413, 0.111212, 0.085678, 0.085067, 0.078208, 0.101018,
    0.103865, 0.081752, 0.082382, 0.108237, 0.093719, 0.084585, 0.079611, 0.082194, 0.084029,
    0.095238, 0.110989, 0.109387
]

# Plotting
plt.figure(figsize=(12, 6))
plt.plot(row1, label='Row 1', marker='o')
plt.plot(row2, label='Row 2', marker='x')
plt.xlabel('Shingle index along the row')
plt.ylabel('Shingle width (m)')
plt.title('Shingle Width Distribution in Roof Rows')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()