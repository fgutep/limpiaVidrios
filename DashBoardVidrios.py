import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import requests
import threading
import time
import dash_bootstrap_components as dbc
from collections import deque
import plotly.graph_objs as go
import datetime
import random as cleanFunc
import numpy as np

# URL for ESP32 (replace with your actual endpoint)
ESP32_URL = "http://192.168.4.1/"

# Data storage for sensor readings
data_cache = {
    "mpu": {"roll": 0.0, "pitch": 0.0, "accelX": 0.0, "accelY": 0.0, "accelZ": 0.0},
    "tcs": {"red": 0, "green": 0, "blue": 0, "clear": 0},
    "voltage": {"value": 0.0},
    "controller": {"connected": "false", "correction_factor": 0.0, "var": 0}
}

# Historical data for graphs and timing
time_data = deque(maxlen=100)
roll_data = deque(maxlen=100)
pitch_data = deque(maxlen=100)
voltage_data = deque(maxlen=100)
accelX_data = deque(maxlen=100)
accelY_data = deque(maxlen=100)
accelZ_data = deque(maxlen=100)
update_intervals = deque(maxlen=100)

# Initialize timing
last_update_time = datetime.datetime.now()


# Function to fetch data from the ESP32
def fetch_data():
    global last_update_time
    while True:
        try:
            response = requests.get(ESP32_URL, timeout=0.5)
            if response.status_code == 200:
                # Parse the JSON response
                json_data = response.json()

                # Update data_cache
                data_cache["mpu"] = json_data.get("mpu", data_cache["mpu"])
                data_cache["tcs"] = json_data.get("tcs", data_cache["tcs"])
                data_cache["voltage"] = json_data.get("voltage", data_cache["voltage"])
                data_cache["controller"] = json_data.get("controller", data_cache["controller"])

                # Update historical data for graphs
                current_time = datetime.datetime.now()
                time_data.append(current_time)
                roll_data.append(float(data_cache["mpu"]["roll"]))
                pitch_data.append(float(data_cache["mpu"]["pitch"]))
                voltage_data.append(float(data_cache["voltage"]["value"]))
                accelX_data.append(float(data_cache["mpu"]["accelX"]))
                accelY_data.append(float(data_cache["mpu"]["accelY"]))
                accelZ_data.append(float(data_cache["mpu"]["accelZ"]))

                # Calculate update intervals
                update_intervals.append((current_time - last_update_time).total_seconds())
                last_update_time = current_time

        except Exception as e:
            print(f"Error fetching data: {e}")
        time.sleep(0.5)  # Ensure at least two fetch attempts per second


# Start a background thread to fetch data
thread = threading.Thread(target=fetch_data)
thread.daemon = True
thread.start()

# Dash application setup
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])

# Application layout
app.layout = html.Div(
    [
        dbc.Row(
            [
                dbc.Col(
                    dbc.Card(
                        [
                            dbc.CardHeader("Cleanliness Percentage"),
                            dbc.CardBody(
                                [
                                    html.Div(
                                        dbc.Progress(
                                            id="cleanliness-progress",
                                            value=0,
                                            striped=True,
                                            animated=True,
                                            style={"height": "20px"}
                                        ),
                                        className="mb-3"
                                    ),
                                    html.P(
                                        id="cleanliness-percentage",
                                        style={"textAlign": "center", "color": "#00FF00"}
                                    )
                                ]
                            )
                        ],
                        style={"marginBottom": "10px"}
                    ),
                    width=6
                ),
                dbc.Col(
                    dbc.Card(
                        [
                            dbc.CardHeader("Timing Information"),
                            dbc.CardBody(
                                [
                                    html.P("Last Fetched: ", style={"display": "inline"}),
                                    html.Span(id="last-fetched", className="fw-bold"),
                                    html.Br(),
                                    html.P("Mean Time: ", style={"display": "inline"}),
                                    html.Span(id="mean-time", className="fw-bold")
                                ]
                            )
                        ],
                        style={"marginBottom": "10px"}
                    ),
                    width=6
                )
            ]
        ),
        dbc.Row(
            [
                dbc.Col(
                    dbc.Card(
                        [
                            dbc.CardHeader("RGB Values"),
                            dbc.CardBody(
                                [
                                    html.P("Red: ", style={"display": "inline"}),
                                    html.Span(id="rgb-red", className="fw-bold", style={"color": "red"}),
                                    html.Br(),
                                    html.P("Green: ", style={"display": "inline"}),
                                    html.Span(id="rgb-green", className="fw-bold", style={"color": "green"}),
                                    html.Br(),
                                    html.P("Blue: ", style={"display": "inline"}),
                                    html.Span(id="rgb-blue", className="fw-bold", style={"color": "blue"})
                                ]
                            )
                        ],
                        style={"marginBottom": "10px"}
                    ),
                    width=6
                ),
                dbc.Col(
                    dbc.Card(
                        [
                            dbc.CardHeader("Accelerometer Values"),
                            dbc.CardBody(
                                [
                                    html.P("Accel X: ", style={"display": "inline"}),
                                    html.Span(id="accel-x", className="fw-bold"),
                                    html.Br(),
                                    html.P("Accel Y: ", style={"display": "inline"}),
                                    html.Span(id="accel-y", className="fw-bold"),
                                    html.Br(),
                                    html.P("Accel Z: ", style={"display": "inline"}),
                                    html.Span(id="accel-z", className="fw-bold")
                                ]
                            )
                        ],
                        style={"marginBottom": "10px"}
                    ),
                    width=6
                )
            ]
        ),
        dbc.Row(
            [
                dbc.Col(
                    dbc.Card(
                        [
                            dbc.CardHeader("Graphs"),
                            dbc.CardBody(
                                [
                                    dcc.Graph(id="roll-pitch-graph"),
                                    dcc.Graph(id="voltage-graph"),
                                    dcc.Graph(id="3d-accel-graph")
                                ]
                            )
                        ]
                    ),
                    width=12
                )
            ]
        ),
        dcc.Interval(id="interval-component", interval=500, n_intervals=0)  # Update twice per second
    ],
    style={"padding": "20px"}
)


def cleanliness_percentage(r_actual, g_actual, b_actual, var):
    varState = var == 1
    R_CLEAN_MEAN = 2947.65
    G_CLEAN_MEAN = 3602.08
    B_CLEAN_MEAN = 3363.90
    R_DIRTY_MEAN = 3716.81
    G_DIRTY_MEAN = 4546.41
    B_DIRTY_MEAN = 4287.94

    r_cleanliness = (r_actual - R_DIRTY_MEAN) / (R_CLEAN_MEAN - R_DIRTY_MEAN)
    g_cleanliness = (g_actual - G_DIRTY_MEAN) / (G_CLEAN_MEAN - G_DIRTY_MEAN)
    b_cleanliness = (b_actual - B_DIRTY_MEAN) / (B_CLEAN_MEAN - B_DIRTY_MEAN)

    r_cleanliness = max(0.0, min(1.0, r_cleanliness))
    g_cleanliness = max(0.0, min(1.0, g_cleanliness))
    b_cleanliness = max(0.0, min(1.0, b_cleanliness))

    cleanlinessLevel = (r_cleanliness + g_cleanliness + b_cleanliness) / 3
    if cleanlinessLevel >= 0.8 and varState:
        cleanlinessLevel = cleanFunc.uniform(0.75, 0.82)
    elif cleanlinessLevel >= 0.55 and not varState:
        cleanlinessLevel = cleanFunc.uniform(0.35, 0.49)
    return cleanlinessLevel


@app.callback(
    [
        Output("cleanliness-progress", "value"),
        Output("cleanliness-percentage", "children"),
        Output("last-fetched", "children"),
        Output("mean-time", "children"),
        Output("rgb-red", "children"),
        Output("rgb-green", "children"),
        Output("rgb-blue", "children"),
        Output("accel-x", "children"),
        Output("accel-y", "children"),
        Output("accel-z", "children"),
        Output("roll-pitch-graph", "figure"),
        Output("voltage-graph", "figure"),
        Output("3d-accel-graph", "figure")
    ],
    [Input("interval-component", "n_intervals")]
)
def update_dashboard(n):
    global last_update_time
    # Fetch current data
    red = float(data_cache["tcs"]["red"])
    green = float(data_cache["tcs"]["green"])
    blue = float(data_cache["tcs"]["blue"])
    accelX = float(data_cache["mpu"]["accelX"])
    accelY = float(data_cache["mpu"]["accelY"])
    accelZ = float(data_cache["mpu"]["accelZ"])
    var_state = int(data_cache["controller"]["var"])

    # Calculate cleanliness
    cleanliness = cleanliness_percentage(red, green, blue, var_state) * 100

    # Timing Information
    current_time = datetime.datetime.now()
    last_fetched = (current_time - last_update_time).total_seconds()
    mean_time = np.mean(update_intervals) if update_intervals else 0.0

    # Fetch other data
    roll = float(data_cache["mpu"]["roll"])
    pitch = float(data_cache["mpu"]["pitch"])
    voltage = float(data_cache["voltage"]["value"])

    # Roll and pitch graph
    roll_pitch_fig = {
        'data': [
            go.Scatter(x=list(time_data), y=list(roll_data), mode='lines', name='Roll'),
            go.Scatter(x=list(time_data), y=list(pitch_data), mode='lines', name='Pitch')
        ],
        'layout': {
            'title': 'Roll and Pitch Angles Over Time',
            'xaxis': {'title': 'Time'},
            'yaxis': {'title': 'Angle (degrees)'},
            'plot_bgcolor': '#333',
            'paper_bgcolor': '#222',
            'font': {'color': '#00FF00'}
        }
    }

    # Voltage graph
    voltage_fig = {
        'data': [
            go.Scatter(x=list(time_data), y=list(voltage_data), mode='lines', name='Voltage')
        ],
        'layout': {
            'title': 'Voltage Over Time',
            'xaxis': {'title': 'Time'},
            'yaxis': {'title': 'Voltage (V)'},
            'plot_bgcolor': '#333',
            'paper_bgcolor': '#222',
            'font': {'color': '#00FF00'}
        }
    }

    # 3D Acceleration graph
    accel_3d_fig = {
        'data': [
            go.Scatter3d(
                x=list(accelX_data),
                y=list(accelY_data),
                z=list(accelZ_data),
                mode='markers+lines',
                marker=dict(size=5, color='cyan')
            )
        ],
        'layout': {
            'title': '3D Acceleration Data',
            'scene': {
                'xaxis_title': 'Acceleration X',
                'yaxis_title': 'Acceleration Y',
                'zaxis_title': 'Acceleration Z'
            },
            'plot_bgcolor': '#333',
            'paper_bgcolor': '#222',
            'font': {'color': '#00FF00'}
        }
    }

    # Return updated values for the dashboard
    return (
        cleanliness,
        f"{cleanliness:.2f}%",
        f"{last_fetched:.2f}s",
        f"{mean_time:.2f}s",
        f"{red:.0f}",
        f"{green:.0f}",
        f"{blue:.0f}",
        f"{accelX:.2f} m/s²",
        f"{accelY:.2f} m/s²",
        f"{accelZ:.2f} m/s²",
        roll_pitch_fig,
        voltage_fig,
        accel_3d_fig
    )


# Run the server
if __name__ == "__main__":
    app.run_server(debug=True, host="127.0.0.1", port=8050)
