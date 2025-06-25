

# Shingle Optimization Project

## Project Structure

```bash
.
├── CMakeLists.txt              
├── INSTALL.md                      # Installation instructions
├── README.md                   
├── config/                     
│   └── config.txt                  # Configuration parameters
├── data/                       
│   ├── export/                     # Exported data
│   ├── scans/                      # Input scan data
│   └── tests/                      # Test datasets
├── include/                        # Header files
│   ├── custom_types.hpp        
│   ├── GeometryProcessor.hpp   
│   └── PointCloudProcessor.hpp 
├── output/                     
│   └── shingle_widths.csv      
├── src/                            # Source files
│   ├── GeometryProcessor.cpp   
│   ├── PointCloudProcessor.cpp 
│   └── main.cpp                
├── tools/                          # Utility tools
│   └── analyze_shingle_widths.cpp  # Tool for analyzing shingle widths

```



### Flowchart

    mmdc -i point_cloud_pipeline.mmd -o point_cloud_pipeline.png -w 1920 -H 1080 --scale 2 

    














