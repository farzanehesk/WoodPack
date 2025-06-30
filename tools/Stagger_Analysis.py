import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to ensure plot saves
import matplotlib.pyplot as plt
import numpy as np

# Data for rows, row lengths, and staggers

row1_s=[39.705, 46.606, 66.337, 53.306, 9.554] 
row1=[163.785, 178.339, 196.018, 119.157, 140.261] 
row1_length=[809.560]


row2_s=[48.914, 78.156, 34.663, 3.242] 
row2=[212.698, 207.581, 205.356, 171.682] 
row2_length=[806.318]

row3_s=[80.905, 92.782, 87.919, 30.057, 4.235] 
row3=[131.793, 170.687, 202.718, 147.494, 145.861] 
row3_length=[810.553]


row4_s=[68.149, 88.739, 72.808, 31.659, 0.885] 
row4=[199.942, 191.277, 186.788, 106.344, 115.087] 
row4_length=[811.437]


row5_s=[80.544, 91.671, 84.083, 31.129, 1.027] 
row5=[119.398, 169.214, 183.690, 180.920, 145.189] 
row5_length=[810.411]


row6_s=[63.532, 73.676, 69.134, 30.259, 1.300] 
row6=[182.930, 179.358, 179.147, 142.045, 116.230] 
row6_length=[811.711]


row7_s=[91.277, 90.269, 88.859, 67.614, 39.851, 12.989] 
row7=[91.653, 178.546, 177.949, 167.719, 104.465, 89.368] 
row7_length=[824.700]


row8_s=[73.091, 59.287, 45.140, 38.305, 14.364] 
row8=[164.744, 164.742, 163.802, 160.885, 144.163] 
row8_length=[810.336]


row9_s=[36.660, 81.361, 82.425, 78.887, 30.267, 4.089] 
row9=[128.084, 120.040, 160.786, 160.264, 112.265, 117.984] 
row9_length=[814.424]


row10_s=[32.119, 66.151, 59.294, 50.412, 47.194, 18.589] 
row10=[160.203, 154.073, 153.929, 151.382, 109.047, 89.379] 
row10_length=[833.014]


row11_s=[33.206, 60.685, 71.893, 72.146, 42.307, 18.584] 
row11=[126.997, 126.593, 142.722, 141.038, 121.544, 140.535] 
row11_length=[814.429]


row12_s=[46.663, 47.265, 44.255, 32.314, 5.187] 
row12=[173.660, 178.388, 144.048, 133.485, 167.662] 
row12_length=[809.242]


row13_s=[80.022, 82.739, 54.190, 57.978, 44.297, 5.052] 
row13=[93.638, 159.761, 149.840, 147.836, 119.804, 128.416] 
row13_length=[814.294]



#######################################


# Combine into final lists
rows = [
    [0] + row1, [0] + row2, [0] + row3, [0] + row4, [0] + row5,
    [0] + row6, [0] + row7, [0] + row8, [0] + row9, [0] + row10,
    [0] + row11, [0] + row12, [0] + row13
]

row_lengths = [
    row1_length[0], row2_length[0], row3_length[0], row4_length[0], row5_length[0],
    row6_length[0], row7_length[0], row8_length[0], row9_length[0], row10_length[0],
    row11_length[0], row12_length[0], row13_length[0]
]

staggers = [
    [0] + row1_s, [0] + row2_s, [0] + row3_s, [0] + row4_s, [0] + row5_s,
    [0] + row6_s, [0] + row7_s, [0] + row8_s, [0] + row9_s, [0] + row10_s,
    [0] + row11_s, [0] + row12_s, [0] + row13_s
]





# Colors for each row (13 colors)
colors = [
    'blue', 'orange', 'green', 'purple', 'red', 'cyan', 'magenta',
    'lime', 'pink', 'teal', 'brown', 'navy', 'olive'
]

# Target length (80 cm = 800 mm)
target_length = 800

# Gap between shingles (3 mm)
gap = 3

# Calculate stagger positions and adjust staggers
stagger_positions = []
adjusted_staggers = []
for i, (row, row_length) in enumerate(zip(rows, row_lengths)):
    # Calculate stagger positions (left edge of shingles 2, 3, ..., n + gap/2, plus row_length for offset)
    n_shingles = len(row)
    cum_widths = [0]  # Start at 0
    for j in range(n_shingles):
        cum_widths.append(cum_widths[-1] + row[j] + (gap if j < n_shingles-1 else 0))
    # Stagger positions at gaps (left edge of next shingle + gap/2)
    stag_pos = [cum_widths[j] + gap/2 for j in range(1, n_shingles)]
    # Use all staggers, replace or append offset
    stagger_data = staggers[i].copy()  # Use all provided staggers
    offset = 800 - row_length  # Negative offset for excess length
    if len(stagger_data) == n_shingles:  # If staggers match shingles, replace last
        stagger_data[-1] = max(offset, -40)
    else:  # Otherwise, append offset
        stagger_data.append(max(offset, -40))
    # Ensure x and y have same length
    n_staggers = len(stagger_data)
    x_positions = stag_pos[:n_staggers-1] + [row_length]  # Last point at row_length
    stagger_positions.append(x_positions)
    adjusted_staggers.append(stagger_data)
    
    # Debug print to verify stagger values and positions
    print(f"Row {i+1}: Staggers = {stagger_data}, X = {x_positions}")

# Define default axis bounds
default_y_max = 90  # Default y-axis upper bound (mm)
default_x_max = max(row_lengths) + 50  # Max row length + padding

# Calculate actual maximum stagger value
max_stagger = max(max(abs(s) for s in stagger) for stagger in adjusted_staggers)  # Max absolute stagger

# Adjust y-axis and x-axis bounds
y_max = max(default_y_max, max_stagger + 10)  # Add padding
x_max = max(default_x_max, max(row_lengths) + 10)  # Max row length + padding

# Create the figure
plt.figure(figsize=(14, 6))

# Plot a curve for each row
for i, (stagger_data, x_positions, color) in enumerate(zip(adjusted_staggers, stagger_positions, colors), 1):
    n_staggers = len(stagger_data)
    n_positions = len(x_positions)
    
    # Verify lengths match
    if n_staggers != n_positions:
        print(f"Error: Row {i} x has {n_positions} elements, y has {n_staggers} elements")
        continue
    
    # Plot all staggers with circle markers
    plt.plot(x_positions, stagger_data, marker='o', linestyle='-', label=f'Row {i}', color=color, linewidth=2, markersize=8, alpha=0.7)

# Add vertical line at 800 mm
plt.axvline(x=target_length, color='black', linestyle='--', linewidth=1, label='800 mm Target Length')

# Add horizontal line at 30 mm
plt.axhline(y=30, color='black', linestyle='--', linewidth=1, label='30 mm Threshold')

# Shade region between 0 mm and 30 mm (unacceptable stagger) with gray hatching
plt.fill_between(
    x=[0, x_max], 
    y1=0, 
    y2=30, 
    color='gray', 
    hatch='/', 
    alpha=0.2, 
    label='Unacceptable Stagger (0–30 mm)'
)

# Shade region between 30 mm and 50 mm
plt.fill_between(
    x=[0, x_max], 
    y1=30, 
    y2=50, 
    color='yellow', 
    alpha=0.2, 
    label='Acceptable Stagger (30–50 mm)'
)

# Shade region above 50 mm
plt.fill_between(
    x=[0, x_max], 
    y1=50, 
    y2=y_max, 
    color='green', 
    alpha=0.2, 
    label='Optimal Stagger (> 50 mm)'
)


plt.xlim(0, x_max)
plt.ylim(-40, y_max)

plt.gca().set_aspect('auto')

plt.yticks(list(plt.yticks()[0]) + [30])


# Customize the plot
plt.xlabel('Stagger Position (mm)')
plt.ylabel('Stagger Value (mm)')
plt.title(f'Stagger Values Across Rows 1 to {len(adjusted_staggers)} (Last Shingle, Offset from 800 mm)')
plt.grid(True, alpha=0.3)
plt.legend(
    fontsize=6,         # smaller font size
    markerscale=0.6,    # smaller markers
    loc='center left',  # vertically centered on left side of the bbox
    bbox_to_anchor=(1.02, 0.75),  # place legend just outside right border
    borderaxespad=0     # no extra padding between axes and legend box
)
plt.xlim(0, x_max)  # Full range from 0 to max row length + padding
plt.ylim(-40, y_max)  # Cap negative staggers at -40 mm
plt.tight_layout(rect=[0, 0, 0.85, 1])  # leave space on right for legend
plt.savefig('Stagger_Analysis.png', bbox_inches='tight')


plt.show()