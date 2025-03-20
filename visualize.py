# Creates and displays graphs of JuliaSim results
import csv
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import pandas as pd
import numpy as np 
import os


def add_temp_legend(values, element, ax):
    np.unique(values)[::-1].sort()
    colors = [ element.cmap(element.norm(value)) for value in values]
    # create a patch (proxy artist) for every color 
    patches = [ mpatches.Patch(color=colors[i], label="{l} violations".format(l=values[i]) ) for i in range(len(values)) ]
        
    ax.legend(handles=patches, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)

def heatmap(df: pd.DataFrame, x_key: str, y_key: str, title:str, **kwargs):
    df = df.groupby([x_key,y_key], as_index=False).agg({'is_violation': np.sum})
    df = df.pivot(columns=x_key, index=y_key, values="is_violation")
    
    fig, ax = plt.subplots()
    im = ax.imshow(df.to_numpy(), **kwargs)
    ax.set_title(title)
    ax.set_xlabel(x_key)
    ax.set_ylabel(y_key)
    
    ax.set_xticks(np.arange(0, len(df.columns), 1))
    ax.set_yticks(np.arange(0, len(df.index), 1))
    ax.set_xticklabels(list(map(lambda v: round(v, 2), df.columns)))
    ax.set_yticklabels(list(map(lambda v: round(v, 2), df.index)))
    
    values = np.unique(df.to_numpy().flatten())
    
    add_temp_legend(values, im, ax)
        
def isolate(df: pd.DataFrame, cols: dict): # keys:list[str], values:list
    exp = None
    for k, v in cols.items():
        if exp is None:
            exp = df[k] == v
        else:
            exp &= df[k] == v
            
    return df[exp]

def plot_point_cloud(df: pd.DataFrame, x_key: str, y_key: str, z_key: str, title: str, point_radius: float = None):
    fig = plt.figure()
    fig.set_figheight(10)
    fig.set_figwidth(10)
    ax = fig.add_subplot(projection='3d')
    df.describe()
    
    x = df[x_key].unique()
    x.sort()
    y = df[y_key].unique()
    y.sort()
    z = df[z_key].unique()
    z.sort()
    
    points = []
    temps = []
    
    
    for zi in z:
        for yi in y:
            for xi in x:
                points.append((xi, yi, zi))
                temps.append(isolate(df, {x_key:xi, y_key:yi, z_key:zi}).size)

    s = point_radius ** 2 if point_radius else None

    # print(len(points), points)

    scat = ax.scatter(*np.array(points).T, c=temps, alpha=0.7, s=s)
    ax.set_xlabel(x_key)
    ax.set_ylabel(y_key)
    ax.set_zlabel(z_key)
    ax.set_title(title)
    
    add_temp_legend(np.unique(np.array(temps[::-1]).flatten()), scat, ax)

def visualize_results(path: str, sim_type: str):
    """Visualize simulation results with different strategies based on simulation type.
    
    Args:
        path (str): Path to the CSV results file
        sim_type (str): Type of simulation ('Vertical', 'Row', or 'Horizontal')
    """
    try: 
        with open(path, "r") as f:
            f.readline()
            dataframe = pd.read_csv(f)
            print(dataframe)
    except Exception as e:
        print(e)
    
    # Common transformations for all simulation types
    all_exps['drone_speed'] = 1/1.444447 * all_exps['drone_speed']
    all_exps['heli_speed'] = 1/1.444447 * all_exps['heli_speed']
    all_exps['is_violation'] = np.where(
        (all_exps['contactLevel'] != 'none'), 
        1, 0)
    all_exps = all_exps.round(4)

    # Visualization strategy based on simulation type
    if sim_type in ["Row", "Horizontal"]:
        # These types have horizontal turn rate
        all_exps['drone_horizontal_turn_rate'] = 180 / np.pi * all_exps['drone_horizontal_turn_rate']
        
        # Horizontal movement visualizations
        heatmap(all_exps, "drone_speed", "drone_horizontal_turn_rate", f"{sim_type} Head-on Collisions")
        heatmap(all_exps, "drone_response_distance", "drone_horizontal_turn_rate", f"{sim_type} Response Distance vs Turn Rate")
        
        # 3D visualization for complex relationships
        violations = isolate(all_exps, {"is_violation": 1})
        plot_point_cloud(violations, "drone_speed", "drone_horizontal_turn_rate", "drone_response_distance",
                        f"{sim_type} Violation Distribution", point_radius=20)
    
    elif sim_type == "Vertical":
        # Vertical movement specific visualizations
        heatmap(all_exps, "drone_speed", "drone_ascent_rate", "Vertical Collision Analysis")
        heatmap(all_exps, "drone_response_distance", "drone_ascent_rate", "Response Distance vs Ascent Rate")
        
        # 3D visualization for vertical scenarios
        violations = isolate(all_exps, {"is_violation": 1})
        plot_point_cloud(violations, "drone_speed", "drone_ascent_rate", "drone_response_distance", "Vertical Violation Distribution", point_radius=20)
    
    # Common visualizations for all types
    heatmap(all_exps, "drone_response_distance", "drone_speed", f"{sim_type} Response Distance Analysis")
    
    plt.show()