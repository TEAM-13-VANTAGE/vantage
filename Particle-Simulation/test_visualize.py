# testing making graphs
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

PATH = os.getcwd() + r"\results\round1\headon-horizontal-results.csv"
with open(PATH, "r") as f:
    f.readline()

all_exps = pd.read_csv(PATH) #.drop_duplicates()
# print(all_exps.shape[0])
all_exps['drone_horizontal_turn_rate'] = 180 / np.pi * all_exps['drone_horizontal_turn_rate']
all_exps['drone_speed'] = 1/1.444447 * all_exps['drone_speed']
all_exps['heli_speed'] = 1/1.444447 * all_exps['heli_speed']
all_exps['is_violation'] = np.where(
    (all_exps['contactLevel'] != 'none'), 
    1, 0)
violations = all_exps[(all_exps['contactLevel'] == 'violation') | (all_exps['contactLevel'] == 'collision')]
# violations.columns
all_exps = all_exps.round(4)

heatmap(all_exps, "drone_speed", "drone_horizontal_turn_rate", "head-on collisions")

all_exps.drone_response_distance.unique()

tmp = all_exps[all_exps['drone_y_pos'] <= 2000] #[all_exps['drone_horizontal_turn_rate'] > 35]
# tmp = isolate(tmp, {"drone_horizontal_turn_rate": 30})
heatmap(all_exps, "drone_response_distance", "drone_speed", "head-on collisions")

colsa = {
    "drone_speed" : 35.0,
    "drone_response_distance" : 1000.0,
    # "heli_speed": 35.0,
    # "drone_y_pos": 1000.0,
    # "drone_horizontal_turn_rate": 10.0,
    "is_violation" : 1,
}
colsb = {
    "drone_speed" : 50.0,
    "drone_response_distance" : 800.0,
    "heli_speed": 35.0,
    "drone_y_pos": 2000,
    "drone_horizontal_turn_rate": 10,
    "is_violation" : 1,
}

# a = isolate(all_exps, colsa)
# b = isolate(all_exps, colsb)
# a


all_exps[(all_exps['drone_response_distance'] == 1200) & (all_exps['drone_speed'] == 20)].describe()


tmp = all_exps#[all_exps['drone_y_pos'] <= 2000]
# tmp = isolate(tmp, {"drone_speed": 20})
heatmap(tmp, "drone_response_distance", "drone_horizontal_turn_rate", "head-on collisions")


# all_exps.columns


# tmp = all_exps[all_exps['drone_y_pos'] <= 2000]
tmp = isolate(all_exps, {"is_violation": 1})
plot_point_cloud(tmp, "drone_speed", "drone_horizontal_turn_rate", "drone_response_distance", "Drone Visual Distance Scatter", point_radius=20)
plt.show()
