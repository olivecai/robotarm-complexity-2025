'''
Oct 14 2025

Plot the Jacobian Accuracy of joint positions tranformed thru fkin into the two projected camera spaces

'''
import matplotlib.pyplot as plt

import ast

with open('jac_accuracy_data_to_plot.txt', 'r') as file:
    lines = [line.strip() for line in file if line.strip()]
    

    for i in range(len(lines)):
        lines[i] = [float(x) for x in lines[i].split()]
    x1_dof2, y1_dof2, x2_dof2, y2_dof2, jac_residuals_dof2, x1_kinova, y1_kinova, x2_kinova, y2_kinova, jac_residuals_kinova = lines

fig, axes = plt.subplots(2, 2, figsize=(10, 10), constrained_layout=True)
ax1 = axes[0, 0]
ax2 = axes[0, 1]
ax3 = axes[1, 0]
ax4 = axes[1, 1]

vmin, vmax = min(jac_residuals_dof2), max(jac_residuals_kinova)

sc1 = ax1.scatter(x1_dof2, y1_dof2, c=jac_residuals_dof2, cmap='viridis',  alpha=0.7)
ax1.set_title('DOF 2 Camera 1 Projection')
ax1.set_xlabel('Camera 1 x-axis')
ax1.set_ylabel('Camera 1 y-axis')
ax1.grid(True)

sc2 = ax2.scatter(x2_dof2, y2_dof2, c=jac_residuals_dof2, cmap='viridis', alpha=0.7)
ax2.set_title('DOF 2 Camera 2 Projection')
ax2.set_xlabel('Camera 2 x-axis')
ax2.set_ylabel('Camera 2 y-axis')
ax2.grid(True)

sc3 = ax3.scatter(x1_kinova, y1_kinova, c=jac_residuals_kinova, cmap='viridis',  alpha=0.7)
ax3.set_title('Kinova Camera 1 Projection')
ax3.set_xlabel('Camera 1 x-axis')
ax3.set_ylabel('Camera 1 y-axis')
ax3.grid(True)

sc4 = ax4.scatter(x2_kinova, y2_kinova, c=jac_residuals_kinova, cmap='viridis', alpha=0.7)
ax4.set_title('Kinova Camera 2 Projection')
ax4.set_xlabel('Camera 2 x-axis')
ax4.set_ylabel('Camera 2 y-axis')
ax4.grid(True)

# shared colorbar
cbar = fig.colorbar(sc3, ax=[ax1, ax2, ax3, ax4])
cbar.set_label('Color Value')

fig.suptitle('Projected Camera View of Workspace', fontsize=14)

plt.show()
   