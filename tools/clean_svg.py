import xml.etree.ElementTree as ET
import math
import sys
import re

try:
    from svg.path import parse_path
except ImportError:
    print("Erreur : La bibliothèque 'svg.path' n'est pas installée.")
    sys.exit(1)

# Permet d'éviter que Python ne rajoute des préfixes "ns0:" de partout
ET.register_namespace('', "http://www.w3.org/2000/svg")

def get_parent(tree, child):
    """Fonction utilitaire pour trouver le parent d'un élément XML"""
    for parent in tree.iter():
        if child in parent:
            return parent
    return None

def optimize_svg_circles(input_file, output_file, tolerance=0.05):
    print(f"Ouverture de {input_file}...")
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    ns = {'svg': 'http://www.w3.org/2000/svg'}
    
    # 1. On cherche maintenant les path, polyline et polygon
    elements_to_check = []
    for tag in ['path', 'polyline', 'polygon']:
        elements_to_check.extend(root.findall(f'.//svg:{tag}', ns))
        elements_to_check.extend(root.findall(f'.//{tag}')) # Au cas où il n'y a pas de namespace explicite

    converted_count = 0

    for elem in elements_to_check:
        points = []
        tag_name = elem.tag.split('}')[-1] # Récupère le nom sans le namespace
        
        # 2A. Extraction des points si c'est un <path>
        if tag_name == 'path':
            d = elem.attrib.get('d', '')
            if not d: continue
            try:
                path = parse_path(d)
                for segment in path:
                    points.append((segment.start.real, segment.start.imag))
                    points.append((segment.end.real, segment.end.imag))
            except Exception:
                continue
                
        # 2B. Extraction des points si c'est une <polyline> ou un <polygon>
        elif tag_name in ['polyline', 'polygon']:
            pts_str = elem.attrib.get('points', '')
            if not pts_str: continue
            
            # Utilise une regex pour extraire tous les nombres (y compris négatifs ou à virgule)
            coords = [float(x) for x in re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', pts_str)]
            
            # On vérifie qu'on a bien des paires (x, y)
            if len(coords) % 2 != 0: continue
            points = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]

        # On ignore les formes avec trop peu de points pour être des cercles discrétisés
        if len(points) < 8:
            continue

        # 3. Calcul de la boîte englobante
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0
        rx = (max_x - min_x) / 2.0
        ry = (max_y - min_y) / 2.0

        if rx == 0 or ry == 0 or abs(rx - ry) / max(rx, ry) > tolerance:
            continue
            
        r = (rx + ry) / 2.0 
        
        # 4. Vérification : tous les points sont-ils sur ce périmètre ?
        is_circle = True
        for x, y in points:
            dist = math.hypot(x - cx, y - cy)
            if abs(dist - r) / r > tolerance:
                is_circle = False
                break
                
        # 5. Remplacement par une vraie balise <circle>
        if is_circle:
            circle_elem = ET.Element('{http://www.w3.org/2000/svg}circle')
            
            # On copie les attributs d'origine (couleur, épaisseur, id...) sauf les coordonnées
            for attr, value in elem.attrib.items():
                if attr not in ['d', 'points']:
                    circle_elem.attrib[attr] = value
                    
            circle_elem.attrib['cx'] = str(round(cx, 4))
            circle_elem.attrib['cy'] = str(round(cy, 4))
            circle_elem.attrib['r'] = str(round(r, 4))
            
            parent = get_parent(root, elem)
            if parent is not None:
                index = list(parent).index(elem)
                parent.insert(index, circle_elem)
                parent.remove(elem)
                converted_count += 1

    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"Terminé ! {converted_count} faux cercles (path/polyline/polygon) ont été convertis.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python ./clean_svg.py <file> [out]")
        sys.exit(1)
        
    input_filename = sys.argv[1]
    output_filename = sys.argv[2] if len(sys.argv) >= 3 else input_filename[:-4] + "_clean_circles.svg"
    
    optimize_svg_circles(input_filename, output_filename)