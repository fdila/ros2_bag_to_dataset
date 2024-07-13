## copy this file in the dataset folder and run it to generate rgb.txt and depth.txt

import os

def create_txt_file(folder_path, output_file):
    with open(output_file, 'w') as f:
        f.write("# timestamp filename\n")
        for filename in os.listdir(folder_path):
            if os.path.isfile(os.path.join(folder_path, filename)):
                timestamp = os.path.splitext(filename)[0]
                relative_path = os.path.join(folder_path, filename)
                f.write(f"{timestamp} {relative_path}\n")


folder_path = "depth"
output_file = "depth.txt"
create_txt_file(folder_path, output_file)

folder_path = "rgb"
output_file = "rgb.txt"
create_txt_file(folder_path, output_file)

