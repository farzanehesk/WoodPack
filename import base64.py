import base64
import io
import requests
from PIL import Image
import matplotlib.pyplot as plt

def mm(graph):
    graphbytes = graph.encode("utf8")
    base64_bytes = base64.urlsafe_b64encode(graphbytes)
    base64_string = base64_bytes.decode("ascii")
    img = Image.open(io.BytesIO(requests.get('https://mermaid.ink/img/' + base64_string).content))
    plt.imshow(img)
    plt.axis('off')
    plt.savefig('flowchart.png', dpi=1200)

mm("""graph TD
    A[Raw PLY Files]:::io --> B{<b>loadPointClouds</b><br>Import and align<br>point clouds}
    B --> C{<b>processPointClouds</b><br>Refine, remove plane,<br>merge point clouds}
    C --> D{<b>EuclideanClustering</b><br>Segment into shingle<br>clusters (DBSCAN)}
    D --> E{<b>computeOrientedBoundingBoxesWithClouds</b><br>Calculate oriented<br>bounding boxes}
    E --> F[Shingle Bounding Boxes]:::io
    classDef process fill:#87cefa,stroke:#4682b4,stroke-width:2px,rx:5px,ry:5px;
    classDef io fill:#d3d3d3,stroke:#666,stroke-width:2px,rx:3px,ry:3px;
    class B,C,D,E process;
    class A,F io;
""")