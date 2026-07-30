### Canvas Version 1.0
## 🛠️ Installation

This project is built in Python. We recommend using a virtual environment.

1. Clone this repository to your computer.
2. Open your terminal or command prompt in the project folder.
3. Install the required dependencies by running:

```bash
pip install -r requirements.txt
```
## 🧠 How It Works (The Simple Explanation)

The software is split into two main steps to keep the math fast and efficient:

Step 1: The Geometry Engine: It reads the shape of your canvas and any holes (obstacles) inside it. It shrinks the boundaries so the robot's tool doesn't spill over the edges, and then mathematically slices the area into straight, safe, parallel paint strokes.

Step 2: The Routing Engine: It takes those unorganized paint strokes and treats them like a puzzle. Using Google OR-Tools, it calculates the absolute shortest path to connect all the strokes together without wasting time flying through the air.

## 🚀 How to Run the Code
Edit your canvas: Open canvas.json and set your canvas coordinates, obstacle coordinates, tool diameter, and the direction the robot will approach from (e.g., "West").

Run the Geometry Engine: Open and run geometry_engine.ipynb. This generates the safe paint strokes and saves them to generated_swaths.json.

Run the Routing Engine: Open and run routing_engine.ipynb. This finds the fastest path and saves the instructions to optimized_path.json.
