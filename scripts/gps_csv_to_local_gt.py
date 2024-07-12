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

df['tx'], df['ty'] = zip(*df.apply(lambda row: latlon_to_local_coordinates(row['lat'], row['lon'], origin), axis=1))

df['timestamp'] = df['sec'].astype(str) + '.' + df['nsec'].astype(str).str[:4]
df['tz'] = 0.0
df['qx'] = 0.0
df['qy'] = 0.0
df['qz'] = 0.0
df['qw'] = 1.0

output_columns = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
with open(output_file, 'w') as f:
    f.write("# timestamp tx ty tz qx qy qz qw\n")
df[output_columns].to_csv(output_file, mode='a', sep=' ', index=False, header=False)


plt.figure(figsize=(10, 6))
plt.plot(df['tx'].values, df['ty'].values, marker='o', linestyle='-', color='b', linewidth=0.4, markersize=0.8)
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Trajectory')
plt.grid(True)
plt.show()
