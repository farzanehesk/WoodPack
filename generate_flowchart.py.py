import base64
import io
import requests
from PIL import Image
import matplotlib.pyplot as plt

def mm(graph):
    graphbytes = graph.encode("utf8")
    base64_bytes = base64.urlsafe_b64encode(graphbytes)
    base64_string = base64_bytes.decode("ascii")
    url = 'https://mermaid.ink/img/' + base64_string
    response = requests.get(url)
    
    # Check response status and content
    if response.status_code != 200:
        print(f"HTTP Error: {response.status_code}")
        print(response.text)
        return
    if 'image' not in response.headers.get('Content-Type', ''):
        print(f"Invalid Content-Type: {response.headers.get('Content-Type')}")
        print(response.text)
        return
    
    try:
        img = Image.open(io.BytesIO(response.content))
        plt.imshow(img)
        plt.axis('off')
        plt.savefig('flowchart.png', dpi=1200)
    except Exception as e:
        print(f"Image Error: {e}")
        print(response.content.decode('utf-8', errors='ignore'))

# Use simplified Mermaid code to avoid parsing issues
mm("""graph LR
    A[Raw PLY files]:::io --> B[Load point clouds\\nImport and align scans]
    B --> C[Process point clouds\\nRefinement, plane removal, merging]
    C --> D[Shingle segmentation\\nEuclidean clustering using DBSCAN]
    D --> E[Bounding box computation\\nOriented bounding boxes OBB]
    E --> F[Shingle bounding boxes]:::io

    classDef process fill:#FFFFFF,stroke:#000000,stroke-width:1.5px,font-weight:bold;
    classDef io fill:#C0C0C0,stroke:#000000,stroke-width:1.5px,font-weight:bold;
    class B,C,D,E process;
    class A,F io;
""")