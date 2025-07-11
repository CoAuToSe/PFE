# grid_selection.py
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from PIL import Image

def select_grid_areas(image_path, M, N):
    """
    Select grid areas using mouse clicks and keyboard inputs.

    Parameters:
    - image_path (str): Path to the image file
    - M (int): Number of rows to divide the image into
    - N (int): Number of columns to divide the image into

    Returns:
    - (list, list, list): Returns lists of obstacle locations, charging stations, scan areas, and free flight zones
    """
    # Load the image
    image = Image.open(image_path)

    # Initialize sets for different areas
    C = [(i, j) for i in range(M) for j in range(N)]  # Set of all grid points
    Obstacles = []  # Obstacle positions
    C_stations = []  # Charging station positions
    S = []  # Scan area positions
    F = C[:]  # Free flight zones (initially all points, updated later based on obstacles)

    # Global variable to track the current selection type: 1=Obstacle, 2=Charging Station, 3=Scan Area
    current_selection = 1  # Default is Obstacle

    # Create figure and axes for the image display
    fig, ax = plt.subplots()

    def on_click(event):
        """Handle mouse click events to record clicks as obstacle, charging station, or scan area."""
        if event.xdata is None or event.ydata is None:
            return

        # Calculate the grid cell corresponding to the click position
        grid_width = image.width / M
        grid_height = image.height / N

        # Calculate row and column indices based on click coordinates
        col = int(event.xdata // grid_width)
        row = N - int(event.ydata // grid_height) - 1

        # Add the clicked grid cell to the respective set
        nonlocal Obstacles, C_stations, S, F

        if (col, row) in C:  # Ensure the click is within the image boundaries
            if current_selection == 1:
                # Obstacle selection
                if (col, row) not in Obstacles:
                    Obstacles.append((col, row))
                else:
                    # Remove if already in the obstacle set (toggle behavior)
                    Obstacles.remove((col, row))
            elif current_selection == 2:
                # Charging station selection
                if (col, row) not in C_stations:
                    C_stations.append((col, row))
                else:
                    # Remove if already in the charging station set
                    C_stations.remove((col, row))
            elif current_selection == 3:
                # Scan area selection
                if (col, row) not in S:
                    S.append((col, row))
                else:
                    # Remove if already in the scan area set
                    S.remove((col, row))

            # Update free flight zones (all grid points - obstacles)
            F = [point for point in C if point not in Obstacles]
            
            # Redraw the grid with updated selections
            draw_grid()

    def draw_grid():
        """Draw the image and grid with the current selections."""
        ax.clear()  # Clear the previous image
        ax.imshow(image)  # Display the image

        # Draw M rows and N columns grid
        grid_width = image.width / M
        grid_height = image.height / N
        for i in range(M):
            for j in range(N):
                rect = patches.Rectangle((j * grid_width, i * grid_height), grid_width, grid_height, linewidth=1, edgecolor='black', facecolor='none')
                ax.add_patch(rect)
        
        # Draw free flight zones
        for (col, row) in F:
            rect = patches.Rectangle((col * grid_width, (N - row - 1) * grid_height), grid_width, grid_height, linewidth=2, edgecolor='green', facecolor='green', alpha=0.5)
            ax.add_patch(rect)
        
        # Draw obstacles
        for (col, row) in Obstacles:
            rect = patches.Rectangle((col * grid_width, (N - row - 1) * grid_height), grid_width, grid_height, linewidth=2, edgecolor='red', facecolor='red', alpha=0.5)
            ax.add_patch(rect)
        
        # Draw charging stations
        for (col, row) in C_stations:
            rect = patches.Rectangle((col * grid_width, (N - row - 1) * grid_height), grid_width, grid_height, linewidth=2, edgecolor='blue', facecolor='blue', alpha=0.5)
            ax.add_patch(rect)
        
        # Draw scan areas with a capital 'S' in the center of each grid cell
        for (col, row) in S:
            ax.text(col * grid_width + grid_width / 2, (N - row - 1) * grid_height + grid_height / 2, 'S',
                    color='white', fontsize=20, ha='center', va='center', fontweight='bold')

        plt.draw()  # Use draw instead of show to refresh the image

    def on_key(event):
        """Handle keyboard events to switch selection type or complete the selection process."""
        nonlocal current_selection
        if event.key == '1':
            current_selection = 1  # Select Obstacle
            print("Current Selection: Obstacle")
        elif event.key == '2':
            current_selection = 2  # Select Charging Station
            print("Current Selection: Charging Station")
        elif event.key == '3':
            current_selection = 3  # Select Scan Area
            print("Current Selection: Scan Area")
        elif event.key == 'enter':
            # Print all final results
            print("Final Selection Results:")
            print(f"All Grid Points: {C}")
            print(f"Free Flight Zones: {F}")
            print(f"Obstacle Positions: {Obstacles}")
            print(f"Charging Station Positions: {C_stations}")
            print(f"Scan Area Positions: {S}")
            plt.close()  # Close the figure window

    # Draw the initial image and grid
    draw_grid()

    # Connect mouse click and keyboard events to their respective handlers
    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)

    # Display the image and wait for user interaction
    plt.show()

    # Return the lists of obstacle positions, charging stations, scan areas, and free flight zones
    return C_stations, S, F
