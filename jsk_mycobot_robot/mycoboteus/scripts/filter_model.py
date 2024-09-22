#/usr/bin/env python3
import xml.etree.ElementTree as ET
import sys

def remove_elements(file_path, elements_to_remove):
    tree = ET.parse(file_path)
    root = tree.getroot()

    for elem in elements_to_remove:
        for element in root.findall(elem):
            root.remove(element)

    output_file = file_path.replace('.urdf', '_filtered.urdf')
    tree.write(output_file)
    print(f"Filtered model saved as {output_file}")

if __name__ == "__main__":
    urdf_file = sys.argv[1]
    elements_to_ignore = sys.argv[2:]  # Remaining arguments are elements to ignore
    remove_elements(urdf_file, elements_to_ignore)
