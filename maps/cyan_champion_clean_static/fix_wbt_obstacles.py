import os
import re

def clean_and_update_wbt(filepath):
    print(f"Processing {filepath}...")
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Define the exact replacement block for obstacles:
    new_obstacles_block = """Solid {
  translation 1.4438 4.0161 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.45 0
        roughness 0.35
      }
      geometry Box {
        size 0.051 0.2263 0.15
      }
    }
  ]
  name "obstacle_box_0"
  contactMaterial "wall"
  boundingObject Box {
    size 0.051 0.2263 0.15
  }
}
Solid {
  translation 0.0501 0.397 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.45 0
        roughness 0.35
      }
      geometry Box {
        size 0.0508 0.2414 0.15
      }
    }
  ]
  name "obstacle_box_1"
  contactMaterial "wall"
  boundingObject Box {
    size 0.0508 0.2414 0.15
  }
}
Solid {
  translation 3.77 2.421 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.75 0
        roughness 0.25
        metalness 0.1
      }
      geometry Cylinder {
        height 0.15
        radius 0.05
      }
    }
  ]
  name "slalom_pillar_3"
  contactMaterial "wall"
  boundingObject Cylinder {
    height 0.15
    radius 0.05
  }
}
Solid {
  translation -0.528 3.567 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.75 0
        roughness 0.25
        metalness 0.1
      }
      geometry Cylinder {
        height 0.15
        radius 0.025
      }
    }
  ]
  name "slalom_pillar_4"
  contactMaterial "wall"
  boundingObject Cylinder {
    height 0.15
    radius 0.025
  }
}
Solid {
  translation -0.528 3 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.75 0
        roughness 0.25
        metalness 0.1
      }
      geometry Cylinder {
        height 0.15
        radius 0.025
      }
    }
  ]
  name "slalom_pillar_5"
  contactMaterial "wall"
  boundingObject Cylinder {
    height 0.15
    radius 0.025
  }
}
Solid {
  translation -1.701 2.348 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.75 0
        roughness 0.25
        metalness 0.1
      }
      geometry Cylinder {
        height 0.15
        radius 0.05
      }
    }
  ]
  name "slalom_pillar_6"
  contactMaterial "wall"
  boundingObject Cylinder {
    height 0.15
    radius 0.05
  }
}
Solid {
  translation -1.018 1.287 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.75 0
        roughness 0.25
        metalness 0.1
      }
      geometry Cylinder {
        height 0.15
        radius 0.05
      }
    }
  ]
  name "slalom_pillar_7"
  contactMaterial "wall"
  boundingObject Cylinder {
    height 0.15
    radius 0.05
  }
}
Solid {
  translation 2.8644 3.2088 0.075
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0.45 0
        roughness 0.35
      }
      geometry Box {
        size 0.0503 0.2171 0.15
      }
    }
  ]
  name "obstacle_box_8"
  contactMaterial "wall"
  boundingObject Box {
    size 0.0503 0.2171 0.15
  }
}"""

    # Match from "Solid {\n  translation 1.4438 4.0161" up to after obstacle_box_8 "size 0.0503 0.2171 0.12\n  }\n}"
    # Or generically from name "obstacle_box_0" to name "obstacle_box_8"
    start_tag = "Solid {\n  translation 1.4438 4.0161"
    end_tag = "name \"obstacle_box_8\"\n  boundingObject Box {\n    size 0.0503 0.2171 0.12\n  }\n}"
    
    start_idx = content.find(start_tag)
    end_idx = content.find(end_tag)
    
    if start_idx != -1 and end_idx != -1:
        end_idx += len(end_tag)
        content = content[:start_idx] + new_obstacles_block + content[end_idx:]
        print("  Replaced obstacle_box_0..8 block successfully.")
    else:
        print(f"  Failed to find tags: start={start_idx}, end={end_idx}")

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"Saved {filepath}.\n")

clean_and_update_wbt(r'C:\Users\nurmi\Documents\Da Winna\maps\cyan_champion_clean_static\Cyan_Champion_Track_Static.wbt')
clean_and_update_wbt(r'C:\Users\nurmi\Documents\Da Winna\my_project\worlds\Cyan_Champion_Track_Static.wbt')
