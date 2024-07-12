import pandas as pd
from geopy.distance import geodesic
import matplotlib.pyplot as plt

def latlon_to_local_coordinates(lat, lon, origin):
    """
    Convert latitude and longitude to local coordinates (x, y) relative to an origin.
    """
    x = geodesic((origin[0], lon), origin).meters
    y = geodesic((lat, origin[1]), origin).meters
    # Adjust signs based on relative positions
    x *= -1 if lon < origin[1] else 1
    y *= -1 if lat < origin[0] else 1
    return x, y

input_file = '/home/fdila/Desktop/otto/otto/gps.csv'
output_file = '/home/fdila/Desktop/otto/otto/gt_local.csv'
df = pd.read_csv(input_file)

origin = (df['lat'][0], df['lon'][0])

df['x'], df['y'] = zip(*df.apply(lambda row: latlon_to_local_coordinates(row['lat'], row['lon'], origin), axis=1))
df[['sec', 'nsec', 'x', 'y']].to_csv(output_file, index=False)

plt.figure(figsize=(10, 6))
plt.plot(df['x'].values, df['y'].values, marker='o', linestyle='-', color='b', linewidth=0.4, markersize=0.8)
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Trajectory')
plt.grid(True)
plt.show()
