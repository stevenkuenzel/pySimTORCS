from race import Race
from track import Track, import_from_torcs


t : Track = import_from_torcs("Brondehach", 1)
print(t)

import matplotlib.pyplot as plt
import matplotlib.patches as patches

def visualize_race(race, draw_size):
    fig, ax = plt.subplots(figsize=(draw_size/100, draw_size/100))
    ax.set_aspect('equal')
    ax.axis('off')

    track : Track = race.track

    # Draw track segments
    for segment in track.segments:
        if hasattr(segment, 'turn'):
            if (segment.turn == 'Right' and abs(segment.turnAngle) >= track.minTurnRad and
                segment.measuredLength <= Track.MAX_TURN_SEGMENT_LENGTH_SQR):
                color = 'red'
            elif (segment.turn == 'Left' and abs(segment.turnAngle) >= track.minTurnRad and
                  segment.measuredLength <= Track.MAX_TURN_SEGMENT_LENGTH_SQR):
                color = 'blue'
            else:
                color = 'green'
        else:
            color = 'black'

        for line in segment.get_normalized_line_segments(track.x_min, track.x_max, track.y_min, track.y_max):  # Updated line
            x1 = line.from_point.x * draw_size
            y1 = line.from_point.y * draw_size
            x2 = line.to_point.x * draw_size
            y2 = line.to_point.y * draw_size
            ax.plot([x1, y1], [x2, y2], color=color, linewidth=1)

    # Draw cars
    for car in race.cars:
        x = ((car.position.x - race.track.xMin) / (race.track.xMax - race.track.xMin)) * draw_size
        y = ((car.position.y - race.track.yMin) / (race.track.yMax - race.track.yMin)) * draw_size

        # Draw car as a circle
        car_circle = patches.Circle((x, y), 4, edgecolor='blue', facecolor='none')
        ax.add_patch(car_circle)

        # Draw heading
        looking_dir = car.position + car.heading_vector.scale(10.0)
        x_to = ((looking_dir.x - race.track.xMin) / (race.track.xMax - race.track.xMin)) * draw_size
        y_to = ((looking_dir.y - race.track.yMin) / (race.track.yMax - race.track.yMin)) * draw_size
        ax.plot([x, x_to], [y, y_to], color='blue')

    plt.show()

r : Race = Race(t, False)

visualize_race(r, 800)