import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from matplotlib.widgets import Slider

# Configuration for the static shuffler
class StaticShufflerConfig:
    def __init__(self, num_shufflers=1, upper_leg_segment=1.0, lower_leg_segment=1.5, cam_offset=0.3, arc_radius=0.2, arc_angle=20, cam_rotation=0, camshaft_rpm=500):
        self.num_shufflers = num_shufflers
        self.upper_leg_segment = upper_leg_segment
        self.lower_leg_segment = lower_leg_segment
        self.cam_offset = cam_offset
        self.cam_radius = 1.1 * cam_offset
        self.arc_radius = arc_radius
        self.arc_angle = arc_angle
        self.cam_rotation = cam_rotation
        self.camshaft_rpm = camshaft_rpm 

    def calculate_lowest_point_variance_and_stepover(self):

        rightedge = np.pi / self.num_shufflers #angle in rad right before the lowest point switches shufflers

        camlocx = np.sin(rightedge) * self.cam_offset
        camlocy = -1 * np.cos(rightedge) * self.cam_offset 

        dx = camlocx
        dy = -1 * np.sqrt((self.upper_leg_segment**2) - dx**2)

        unitx = dx/self.upper_leg_segment
        unity = dy/self.upper_leg_segment

        # Center point on the arc
        arccenterx = camlocx + (self.lower_leg_segment * unitx) 
        arccentery = camlocy + (self.lower_leg_segment * unity)

        # Center of the arc circle
        centerx = arccenterx - (self.arc_radius * unitx)
        centery = arccentery - (self.arc_radius * unity)
        
        # Angle of leg at transition point in radians
        sigma_rad = np.arctan(unity / unitx)

        # Angle of leg at transition point from lower vertical
        vert_rad = (np.pi / 2) + sigma_rad

        # Angle about arc center that the left tip of the arc is at the transition point from lower vertical
        left_rad =(vert_rad - (self.arc_angle * np.pi / 180 / 2))

        # Calculate the lowest point options on a complete circle
        lowestx = centerx
        lowesty = centery - self.arc_radius

        # Calculate if the arc does not go to the lowest point on the circle
        lowesty2 = centery - (np.cos(left_rad) * self.arc_radius)
        lowestx2 = centerx + (np.sin(left_rad) * self.arc_radius)

        # Determine the lowest point based on x values
        if left_rad > 0: # MAY NEED TO FLIP THE EQUALITY
            lowestx = lowestx2
            lowesty = lowesty2

        usedhalfchord = np.sqrt((arccenterx - lowestx)**2 + (arccentery - lowesty)**2) 

        usedangle = 2*(2* np.arcsin(usedhalfchord/(2*self.arc_radius))) # doubling the chord conversion to reach full angle
        maxarc = self.arc_angle * self.arc_radius * 2 * np.pi # whole foot
        usedarc = usedangle/self.arc_angle *maxarc /(2* np.pi)

        # Stepover calculation
        stepover = self.num_shufflers * ((2 * lowestx) + usedarc)

        # Lowest point variance calculation
        lowest_point_variance = np.abs((-self.cam_offset - self.lower_leg_segment) - lowesty) 
    
        return lowest_point_variance, stepover

    def calculate_speeds(self, stepover):
        feet_per_second = (stepover * self.camshaft_rpm) / 720
        miles_per_hour = feet_per_second * 1.46667
        return feet_per_second, miles_per_hour

# Plot multiple shufflers in a pod
def plot_shuffler_pod(config: StaticShufflerConfig):
    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(left=0.2, bottom=0.45, right=0.4) 
    colors = plt.cm.tab10.colors  # Color map for unique shufflers

    # Initial calculation for display values
    lowest_point_variance, stepover = config.calculate_lowest_point_variance_and_stepover()
    feet_per_second, miles_per_hour = config.calculate_speeds(stepover)

    display_text = fig.text(0.6, 0.7, f"Lowest Point Variance: {lowest_point_variance:.2f}\n360-Degree Stepover: {stepover:.2f}\nFeet per Second: {feet_per_second:.2f}\nMiles per Hour: {miles_per_hour:.2f}",
                            ha="left", va="top", fontsize=10, transform=fig.transFigure)

    def update_display():
        lowest_point_variance, stepover = config.calculate_lowest_point_variance_and_stepover()
        feet_per_second, miles_per_hour = config.calculate_speeds(stepover)
        display_text.set_text(f"Lowest Point Variance: {lowest_point_variance:.2f}\n360-Degree Stepover: {stepover:.2f}\nFeet per Second: {feet_per_second:.2f}\nMiles per Hour: {miles_per_hour:.2f}")

    def draw_pod():
        ax.clear()
        
        cam_shaft_position = np.array([0, 0])
        base_rotation_angle = config.cam_rotation
        angle_step = 360 / config.num_shufflers

        cam_circle = plt.Circle(cam_shaft_position, config.cam_radius, color='lightblue', alpha=0.5, label="Cam")
        ax.add_patch(cam_circle)

        cam_shaft_radius = config.cam_radius * 0.2
        cam_shaft = plt.Circle(cam_shaft_position, cam_shaft_radius, color='darkblue', label="Cam Shaft")
        ax.add_patch(cam_shaft)

        lowest_y = None
        lowest_point_position = None

        for i in range(config.num_shufflers):
            offset_angle = np.deg2rad(base_rotation_angle + i * angle_step)
            offset_x = config.cam_offset * np.cos(offset_angle)
            offset_y = config.cam_offset * np.sin(offset_angle)
            offset_point = cam_shaft_position + np.array([offset_x, offset_y])

            moving_point_x = cam_shaft_position[0]
            vertical_displacement = np.sqrt(config.upper_leg_segment**2 - (offset_point[0] - moving_point_x)**2)
            moving_point_y = offset_point[1] + vertical_displacement
            moving_point_position = np.array([moving_point_x, moving_point_y])

            direction_vector = offset_point - moving_point_position
            direction_unit_vector = direction_vector / np.linalg.norm(direction_vector)
            leg_end = offset_point + direction_unit_vector * config.lower_leg_segment

            arc_center = leg_end - direction_unit_vector * config.arc_radius

            theta1 = -config.arc_angle / 2
            theta2 = config.arc_angle / 2
            for theta in np.linspace(theta1, theta2, num=20):
                angle_rad = np.deg2rad(theta)
                edge_x = arc_center[0] + config.arc_radius * np.cos(angle_rad + np.arctan2(direction_unit_vector[1], direction_unit_vector[0]))
                edge_y = arc_center[1] + config.arc_radius * np.sin(angle_rad + np.arctan2(direction_unit_vector[1], direction_unit_vector[0]))

                if lowest_y is None or edge_y < lowest_y:
                    lowest_y = edge_y
                    lowest_point_position = np.array([edge_x, edge_y])

            color = colors[i % len(colors)]

            ax.plot(*offset_point, 'o', color=color)
            
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

            foot_arc = patches.Arc(arc_center, width=config.arc_radius * 2, height=config.arc_radius * 2,
                                   angle=np.rad2deg(np.arctan2(direction_unit_vector[1], direction_unit_vector[0])),
                                   theta1=theta1, theta2=theta2, color=color)
            ax.add_patch(foot_arc)

            ax.plot(*moving_point_position, 'o', color=color)

        if lowest_point_position is not None:
            ax.plot(*lowest_point_position, 'o', color='black', label="Lowest Point")

        ax.set_aspect('equal')
        ax.set_title("Shuffler Pod")
        ax.set_xlabel("X Position (inches)")
        ax.set_ylabel("Y Position (inches)")
        ax.grid(True)
        ax.legend(loc="upper left", bbox_to_anchor=(1.05, 1))

        ax.set_xlim(-config.lower_leg_segment - 1, config.upper_leg_segment + config.lower_leg_segment + 1)
        ax.set_ylim(-config.lower_leg_segment - 1, config.upper_leg_segment + config.lower_leg_segment + 1)
        
        fig.canvas.draw_idle()

    draw_pod()
    update_display()

    ax_num_shufflers = plt.axes([0.25, 0.4, 0.65, 0.03])
    ax_upper_leg = plt.axes([0.25, 0.25, 0.65, 0.03])
    ax_lower_leg = plt.axes([0.25, 0.2, 0.65, 0.03])
    ax_cam_offset = plt.axes([0.25, 0.15, 0.65, 0.03])
    ax_arc_radius = plt.axes([0.25, 0.1, 0.65, 0.03])
    ax_arc_angle = plt.axes([0.25, 0.05, 0.65, 0.03])
    ax_cam_rotation = plt.axes([0.25, 0.3, 0.65, 0.03])
    ax_camshaft_rpm = plt.axes([0.25, 0.35, 0.65, 0.03])

    slider_num_shufflers = Slider(ax_num_shufflers, 'Number of Shufflers', 1, 6, valinit=config.num_shufflers, valstep=1)
    slider_upper_leg = Slider(ax_upper_leg, 'Upper Leg Segment', 0.5, 3.0, valinit=config.upper_leg_segment)
    slider_lower_leg = Slider(ax_lower_leg, 'Lower Leg Segment', 0.5, 3.0, valinit=config.lower_leg_segment)
    slider_cam_offset = Slider(ax_cam_offset, 'Cam Offset', 0.0, 0.5, valinit=config.cam_offset)
    slider_arc_radius = Slider(ax_arc_radius, 'Arc Radius', 2, 15, valinit=config.arc_radius)
    slider_arc_angle = Slider(ax_arc_angle, 'Arc Angle', 3, 90, valinit=config.arc_angle)
    slider_cam_rotation = Slider(ax_cam_rotation, 'Cam Rotation', 0, 360, valinit=config.cam_rotation)
    slider_camshaft_rpm = Slider(ax_camshaft_rpm, 'Camshaft RPM', 500, 15000, valinit=config.camshaft_rpm, valstep=100)

    def update(val):
        config.num_shufflers = slider_num_shufflers.val
        config.upper_leg_segment = slider_upper_leg.val
        config.lower_leg_segment = slider_lower_leg.val
        config.cam_offset = slider_cam_offset.val
        config.cam_radius = 1.1 * config.cam_offset 
        config.arc_radius = slider_arc_radius.val
        config.arc_angle = slider_arc_angle.val
        config.cam_rotation = slider_cam_rotation.val
        config.camshaft_rpm = slider_camshaft_rpm.val
        draw_pod()
        update_display()

    def updatepod(val):
        config.cam_rotation = slider_cam_rotation.val
        draw_pod()

    def updatedisplay(val):
        config.camshaft_rpm = slider_camshaft_rpm.val
        update_display()

    slider_num_shufflers.on_changed(update)
    slider_upper_leg.on_changed(update)
    slider_lower_leg.on_changed(update)
    slider_cam_offset.on_changed(update)
    slider_arc_radius.on_changed(update)
    slider_arc_angle.on_changed(update)
    slider_cam_rotation.on_changed(updatepod)
    slider_camshaft_rpm.on_changed(updatedisplay)

    plt.show()

# Updated config
updated_config = StaticShufflerConfig(num_shufflers=4, upper_leg_segment=1.0, lower_leg_segment=1.25, cam_offset=0.15, arc_radius=6, arc_angle=12, cam_rotation=0, camshaft_rpm=1500)
plot_shuffler_pod(updated_config)
