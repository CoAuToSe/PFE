import os
import subprocess
import open3d as o3d
import numpy as np
import laspy

def convert_copc_laz_to_xyz(input_file, output_xyz):
    """
    Utilise PDAL pour convertir un fichier .copc.laz en .xyz (lisible par Open3D).
    """
    pipeline = f"""
    {{
        "pipeline": [
            "{input_file}",
            {{
                "type": "writers.text",
                "filename": "{output_xyz}",
                "format": "xyz"
            }}
        ]
    }}
    """
    with open("pipeline.json", "w") as f:
        f.write(pipeline)
    subprocess.run(["pdal", "pipeline", "pipeline.json"])
    os.remove("pipeline.json")

def xyz_to_mesh(xyz_file, output_mesh_file):
    """
    Lit un fichier .xyz, crée un mesh avec Open3D, puis l’exporte en .stl ou .obj.
    """
    # Lecture du fichier XYZ
    xyz = np.loadtxt(xyz_file)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # Génération du maillage (Poisson Surface Reconstruction)
    print("Reconstruction de surface avec Poisson...")
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)

    # Nettoyage (optionnel)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()

    # Export du mesh
    o3d.io.write_triangle_mesh(output_mesh_file, mesh)
    print(f"Mesh exporté vers {output_mesh_file}")

# === Exemple d’utilisation avec 2 fichiers ===
fichiers = ["Générer automatiquement monde Gazebo/LHD_FXX_0642_6846_PTS_O_LAMB93_IGN69.copc.laz", "Générer automatiquement monde Gazebo/LHD_FXX_0642_6847_PTS_O_LAMB93_IGN69.copc.laz"]

for fichier in fichiers:
    base = os.path.splitext(fichier)[0]
    xyz_file = base + ".xyz"
    output_mesh = base + ".obj"  # ou ".stl" selon besoin

    convert_copc_laz_to_xyz(fichier, xyz_file)
    xyz_to_mesh(xyz_file, output_mesh)
