import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from matplotlib.widgets import Slider

# Configuration for the static shuffler
class StaticShufflerConfig:
    def __init__(self, num_shufflers=1, upper_leg_segment=1.0, lower_leg_segment=1.5, cam_offset=0.3, arc_radius=0.2, arc_angle=20, cam_rotation=0):
        self.num_shufflers = num_shufflers
        self.upper_leg_segment = upper_leg_segment
        self.lower_leg_segment = lower_leg_segment
        self.cam_offset = cam_offset
        self.cam_radius = 1.1 * cam_offset
        self.arc_radius = arc_radius
        self.arc_angle = arc_angle
        self.cam_rotation = cam_rotation

    def calculate_lowest_point_variance_and_stepover(self):
        min_y, max_y = None, None
        min_x, max_x = None, None

        for angle in range(0, 360, 10):  # Increment rotation through 360 degrees
            rotation = np.deg2rad(angle)
            base_angle = rotation  # Starting base rotation
            lowest_y = None
            lowest_x = None

            for i in range(self.num_shufflers):
                offset_angle = base_angle + i * (2 * np.pi / self.num_shufflers)
                offset_x = self.cam_offset * np.cos(offset_angle)
                offset_y = self.cam_offset * np.sin(offset_angle)
                offset_point = np.array([offset_x, offset_y])

                moving_point_x = 0
                vertical_displacement = np.sqrt(self.upper_leg_segment**2 - offset_point[0]**2)
                moving_point_y = offset_point[1] + vertical_displacement
                moving_point_position = np.array([moving_point_x, moving_point_y])

                direction_vector = offset_point - moving_point_position
                direction_unit_vector = direction_vector / np.linalg.norm(direction_vector)
                leg_end = offset_point + direction_unit_vector * self.lower_leg_segment

                arc_center = leg_end - direction_unit_vector * self.arc_radius

                # Sample points along the arc edge to find the lowest point
                theta1 = -self.arc_angle / 2
                theta2 = self.arc_angle / 2
                for theta in np.linspace(theta1, theta2, num=20):
                    angle_rad = np.deg2rad(theta)
                    edge_x = arc_center[0] + self.arc_radius * np.cos(angle_rad + np.arctan2(direction_unit_vector[1], direction_unit_vector[0]))
                    edge_y = arc_center[1] + self.arc_radius * np.sin(angle_rad + np.arctan2(direction_unit_vector[1], direction_unit_vector[0]))

                    if lowest_y is None or edge_y < lowest_y:
                        lowest_y = edge_y
                        lowest_x = edge_x

            if min_y is None or lowest_y < min_y:
                min_y = lowest_y
            if max_y is None or lowest_y > max_y:
                max_y = lowest_y

            if min_x is None or lowest_x < min_x:
                min_x = lowest_x
            if max_x is None or lowest_x > max_x:
                max_x = lowest_x

        lowest_point_variance = max_y - min_y if min_y is not None and max_y is not None else 0
        stepover = self.num_shufflers * (max_x - min_x) if min_x is not None and max_x is not None else 0

        return lowest_point_variance, stepover

# Function to plot multiple shufflers in a pod
def plot_shuffler_pod(config: StaticShufflerConfig):
    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(left=0.25, bottom=0.4, right=0.85)  # Adjust to add space for text display on the right
    colors = plt.cm.tab10.colors  # Color map for unique shufflers

    # Initial calculation for display values
    lowest_point_variance, stepover = config.calculate_lowest_point_variance_and_stepover()
    
    # Display for metrics under the legend
    display_text = fig.text(0.8, 0.7, f"Lowest Point Variance: {lowest_point_variance:.2f}\n360-Degree Stepover: {stepover:.2f}",
                            ha="left", va="top", fontsize=10, transform=fig.transFigure)

    def update_display():
        lowest_point_variance, stepover = config.calculate_lowest_point_variance_and_stepover()
        display_text.set_text(f"Lowest Point Variance: {lowest_point_variance:.2f}\n360-Degree Stepover: {stepover:.2f}")

    # Initial drawing function
    def draw_pod():
        ax.clear()
        
        # Define the fixed cam shaft position
        cam_shaft_position = np.array([0, 0])  # Fixed point in space
        base_rotation_angle = config.cam_rotation  # Base rotation for the first offset
        angle_step = 360 / config.num_shufflers  # Equal angular spacing for each shuffler

        # Draw the cam as a larger circle based on calculated radius
        cam_circle = plt.Circle(cam_shaft_position, config.cam_radius, color='lightblue', alpha=0.5, label="Cam")
        ax.add_patch(cam_circle)

        # Draw the cam shaft as a smaller circle at the center
        cam_shaft_radius = config.cam_radius * 0.2  # Smaller radius for cam shaft
        cam_shaft = plt.Circle(cam_shaft_position, cam_shaft_radius, color='darkblue', label="Cam Shaft")
        ax.add_patch(cam_shaft)

        # Initialize to track the lowest y-coordinate among all arcs
        lowest_y = None
        lowest_point_position = None

        # Iterate over each shuffler in the pod
        for i in range(config.num_shufflers):
            offset_angle = np.deg2rad(base_rotation_angle + i * angle_step)
            offset_x = config.cam_offset * np.cos(offset_angle)
            offset_y = config.cam_offset * np.sin(offset_angle)
            offset_point = cam_shaft_position + np.array([offset_x, offset_y])  # Position of cam offset point

            # Fixed x-coordinate for the moving point above cam
            moving_point_x = cam_shaft_position[0]  # Same x-coordinate as cam shaft
            # Calculate y-coordinate to maintain fixed upper leg length
            vertical_displacement = np.sqrt(config.upper_leg_segment**2 - (offset_point[0] - moving_point_x)**2)
            moving_point_y = offset_point[1] + vertical_displacement
            moving_point_position = np.array([moving_point_x, moving_point_y])

            # Calculate the lower leg segment end point
            direction_vector = offset_point - moving_point_position
            direction_unit_vector = direction_vector / np.linalg.norm(direction_vector)
            leg_end = offset_point + direction_unit_vector * config.lower_leg_segment

            # Position the arc so its edge is aligned with the end of the lower leg segment
            arc_center = leg_end - direction_unit_vector * config.arc_radius

            # Sample points along the arc edge to find the actual lowest point on the arc
            theta1 = -config.arc_angle / 2
            theta2 = config.arc_angle / 2
            for theta in np.linspace(theta1, theta2, num=20):
                angle_rad = np.deg2rad(theta)
                edge_x = arc_center[0] + config.arc_radius * np.cos(angle_rad + np.arctan2(direction_unit_vector[1], direction_unit_vector[0]))
                edge_y = arc_center[1] + config.arc_radius * np.sin(angle_rad + np.arctan2(direction_unit_vector[1], direction_unit_vector[0]))

                if lowest_y is None or edge_y < lowest_y:
                    lowest_y = edge_y
                    lowest_point_position = np.array([edge_x, edge_y])

            # Color each shuffler uniquely
            color = colors[i % len(colors)]

            # Draw the offset point on the cam
            ax.plot(*offset_point, 'o', color=color)
            
            # Draw upper and lower leg segments for each shuffler as a single color
            ax.plot(
                [offset_point[0], moving_point_position[0]],
                [offset_point[1], moving_point_position[1]],
                color=color
            )

            ax.plot(
                [offset_point[0], leg_end[0]],
                [offset_point[1], leg_end[1]],
                color=color
            )

            # Draw the arc centered around the leg segment end point
            foot_arc = patches.Arc(arc_center, width=config.arc_radius * 2, height=config.arc_radius * 2,
                                   angle=np.rad2deg(np.arctan2(direction_unit_vector[1], direction_unit_vector[0])),
                                   theta1=theta1, theta2=theta2, color=color)
            ax.add_patch(foot_arc)

            # Draw the moving point above the cam shaft in the color of its shuffler
            ax.plot(*moving_point_position, 'o', color=color)

        # Draw the lowest point on the plot
        if lowest_point_position is not None:
            ax.plot(*lowest_point_position, 'o', color='black', label="Lowest Point")

        # Plot aesthetics
        ax.set_aspect('equal')  # Ensure 1:1 aspect ratio
        ax.set_title("Shuffler Pod")
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.grid(True)
        ax.legend(loc="upper left", bbox_to_anchor=(1.05, 1))  # Move legend outside plot

        # Set limits to make everything visible
        ax.set_xlim(-config.cam_radius - 1, config.cam_radius + 1)
        ax.set_ylim(-config.lower_leg_segment - 1, config.upper_leg_segment + config.lower_leg_segment + 1)
        
        fig.canvas.draw_idle()

    # Draw initial pod and display text
    draw_pod()
    update_display()

    # Create sliders for interactive parameter adjustment
    ax_num_shufflers = plt.axes([0.25, 0.4, 0.65, 0.03])
    ax_upper_leg = plt.axes([0.25, 0.25, 0.65, 0.03])
    ax_lower_leg = plt.axes([0.25, 0.2, 0.65, 0.03])
    ax_cam_offset = plt.axes([0.25, 0.15, 0.65, 0.03])
    ax_arc_radius = plt.axes([0.25, 0.1, 0.65, 0.03])
    ax_arc_angle = plt.axes([0.25, 0.05, 0.65, 0.03])
    ax_cam_rotation = plt.axes([0.25, 0.3, 0.65, 0.03])

    slider_num_shufflers = Slider(ax_num_shufflers, 'Number of Shufflers', 1, 6, valinit=config.num_shufflers, valstep=1)
    slider_upper_leg = Slider(ax_upper_leg, 'Upper Leg Segment', 0.5, 3.0, valinit=config.upper_leg_segment)
    slider_lower_leg = Slider(ax_lower_leg, 'Lower Leg Segment', 0.5, 3.0, valinit=config.lower_leg_segment)
    slider_cam_offset = Slider(ax_cam_offset, 'Cam Offset', 0.0, 1.0, valinit=config.cam_offset)
    slider_arc_radius = Slider(ax_arc_radius, 'Arc Radius', 1, 20, valinit=config.arc_radius)
    slider_arc_angle = Slider(ax_arc_angle, 'Arc Angle', 10, 180, valinit=config.arc_angle)
    slider_cam_rotation = Slider(ax_cam_rotation, 'Cam Rotation', 0, 360, valinit=config.cam_rotation)

    # Update function for sliders
    def update(val):
        config.num_shufflers = slider_num_shufflers.val
        config.upper_leg_segment = slider_upper_leg.val
        config.lower_leg_segment = slider_lower_leg.val
        config.cam_offset = slider_cam_offset.val
        config.cam_radius = 1.1 * config.cam_offset  # Recalculate cam radius based on cam offset
        config.arc_radius = slider_arc_radius.val
        config.arc_angle = slider_arc_angle.val
        config.cam_rotation = slider_cam_rotation.val
        draw_pod()
        update_display()  # Update display text

    # Attach the update function to sliders
    slider_num_shufflers.on_changed(update)
    slider_upper_leg.on_changed(update)
    slider_lower_leg.on_changed(update)
    slider_cam_offset.on_changed(update)
    slider_arc_radius.on_changed(update)
    slider_arc_angle.on_changed(update)
    slider_cam_rotation.on_changed(update)

    plt.show()

# Example usage with updated configuration
updated_config = StaticShufflerConfig(num_shufflers=4, upper_leg_segment=1.0, lower_leg_segment=1.5, cam_offset=0.3, arc_radius=0.2, arc_angle=20, cam_rotation=0)
plot_shuffler_pod(updated_config)