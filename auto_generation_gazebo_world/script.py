import os
import subprocess
import osmnx as ox
import folium
import time
import requests

# ====== CONFIGURATION ======
OSM2WORLD_JAR = "pfe_2025/Générer automatiquement monde Gazebo/OSM2World-latest-bin/OSM2World.jar"
RADIUS_METERS = 300
WORLD_FILENAME = "generated_world.world"
OBJ_FILENAME = "buildings.obj"
OSM_FILENAME = "extracted.osm"
# ============================

def create_map():
    print("Génération de la carte interactive...")
    m = folium.Map(location=[48.7114, 2.2182], zoom_start=15)
    m.add_child(folium.LatLngPopup())
    m.save("select_point_map.html")
    print("✅ Ouvre le fichier 'select_point_map.html' dans ton navigateur, clique quelque part, et copie les coordonnées.")
    print("Puis entre les coordonnées ici (format: lat lon)")
    os.system("xdg-open select_point_map.html")

def get_user_coordinates():
    coords = input("Entrez les coordonnées (lat lon) du point sélectionné : ").strip()
    lat, lon = map(float, coords.split())
    return lat, lon

def extract_osm_data(lat, lon):
    print(f"📡 Extraction des données OSM autour de ({lat}, {lon}) dans un rayon de {RADIUS_METERS} m...")

    overpass_url = "http://overpass-api.de/api/interpreter"
    overpass_query = f"""
    [out:xml];
    (
      way["building"](around:{RADIUS_METERS},{lat},{lon});
      relation["building"](around:{RADIUS_METERS},{lat},{lon});
      node["natural"="tree"](around:{RADIUS_METERS},{lat},{lon});
      way["natural"="wood"](around:{RADIUS_METERS},{lat},{lon});
      way["landuse"="forest"](around:{RADIUS_METERS},{lat},{lon});
      way["leisure"="park"](around:{RADIUS_METERS},{lat},{lon});
      node["highway"="street_lamp"](around:{RADIUS_METERS},{lat},{lon});
      node["amenity"="bench"](around:{RADIUS_METERS},{lat},{lon});
      way["highway"](around:{RADIUS_METERS},{lat},{lon});
      way["natural"="water"](around:{RADIUS_METERS},{lat},{lon});
    );
    (._;>;);
    out body;
    """

    response = requests.get(overpass_url, params={'data': overpass_query})

    if response.status_code == 200:
        with open(OSM_FILENAME, 'w', encoding='utf-8') as f:
            f.write(response.text)
        print(f"✅ Données OSM sauvegardées dans {OSM_FILENAME}")
    else:
        print("❌ Erreur lors de la requête Overpass")
        print(response.text)
        exit(1)

def convert_to_obj():
    print("🔄 Conversion .osm -> .obj avec osm2world...")
    if not os.path.exists(OSM2WORLD_JAR):
        raise FileNotFoundError("❌ osm2world.jar introuvable.")
    subprocess.run(["java", "-jar", OSM2WORLD_JAR, "-i", OSM_FILENAME, "-o", OBJ_FILENAME])

def generate_world_file():
    print("🧾 Génération du fichier .world pour Gazebo...")
    with open(WORLD_FILENAME, "w") as f:
        f.write(f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="osm_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name="osm_buildings">
      <static>true</static>
      <pose>0 0 0 1.5708 0 0</pose> <!-- 90° autour de X (en radians) -->
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://{os.path.abspath(OBJ_FILENAME)}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://{os.path.abspath(OBJ_FILENAME)}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
""")
    print(f"✅ Monde généré dans {WORLD_FILENAME}")

if __name__ == "__main__":
    create_map()
    lat, lon = get_user_coordinates()
    extract_osm_data(lat, lon)
    convert_to_obj()
    generate_world_file()
