import os
import dash
from dash import html, dcc
from dash.dependencies import Input, Output
import dash_bootstrap_components as dbc
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import threading

# Directory to monitor
DIRECTORY_TO_WATCH = "/home/rh/catkin_ws/src/yolov5/runs/detect"

# Global dictionary to hold the count of each bottle type
bottle_counts = {'Great Northern': 0, 'Pale Ale': 0, 'Heineken': 0}
total_folders = 0

# Set up the Dash app
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

app.layout = dbc.Container([
    dbc.Row(dbc.Col(html.H1("Bottle Detection Dashboard", className="text-center my-3"), width=12)),
    dbc.Row(dbc.Col(html.H2(id='total-folders-text', className="text-center text-primary my-3"), width=12)),
    dbc.Row([
        dbc.Col(html.Div(id='live-update-text'), width=12, md=6, className="mb-4")
    ], justify="center", align="center"),
    dcc.Interval(id='interval-component', interval=1000, n_intervals=0)  # Update every second
], fluid=True, className="h-100", style={'height': '100vh'})

@app.callback(
    [Output('live-update-text', 'children'), Output('total-folders-text', 'children')],
    Input('interval-component', 'n_intervals'))
def update_metrics(n):
    brands = [dbc.Card([
                dbc.CardHeader(brand, style={'textAlign': 'center'}),
                dbc.CardBody(html.H4(f"{count}", className="card-title", style={'textAlign': 'center'}))
            ], className="text-center m-2", style={'width': '400px', 'height': '120px'})
            for brand, count in bottle_counts.items()]
    total = f"Total Detected Bottles: {total_folders}"
    return dbc.Row(brands, justify="center"), total

def initialize_counts():
    global total_folders
    for folder in os.listdir(DIRECTORY_TO_WATCH):
        folder_path = os.path.join(DIRECTORY_TO_WATCH, folder)
        if os.path.isdir(folder_path):
            total_folders += 1
            label_file_path = os.path.join(folder_path, 'labels')
            for file in os.listdir(label_file_path):
                if file.endswith('.txt'):
                    with open(os.path.join(label_file_path, file), 'r') as f:
                        lines = f.readlines()
                        for line in lines:
                            first_number = int(line.strip().split()[0])
                            if first_number == 0:
                                bottle_counts['Great Northern'] += 1
                            elif first_number == 1:
                                bottle_counts['Pale Ale'] += 1
                            elif first_number == 2:
                                bottle_counts['Heineken'] += 1

class DetectionHandler(FileSystemEventHandler):
    def on_created(self, event):
        global total_folders
        if event.is_directory:
            label_file_path = os.path.join(event.src_path, 'labels')
            if os.path.exists(label_file_path):
                total_folders += 1
                try:
                    for file in os.listdir(label_file_path):
                        if file.endswith('.txt'):
                            with open(os.path.join(label_file_path, file), 'r') as f:
                                lines = f.readlines()
                                for line in lines:
                                    first_number = int(line.strip().split()[0])
                                    if first_number == 0:
                                        bottle_counts['Great Northern'] += 1
                                    elif first_number == 1:
                                        bottle_counts['Pale Ale'] += 1
                                    elif first_number == 2:
                                        bottle_counts['Heineken'] += 1
                except FileNotFoundError as e:
                    print(f"Error reading files in {label_file_path}: {e}")
            else:
                print(f"Directory not found: {label_file_path}")

observer = Observer()
event_handler = DetectionHandler()
observer.schedule(event_handler, path=DIRECTORY_TO_WATCH, recursive=True)
observer.start()

initialize_counts()

if __name__ == '__main__':
    app.run_server(debug=True)
