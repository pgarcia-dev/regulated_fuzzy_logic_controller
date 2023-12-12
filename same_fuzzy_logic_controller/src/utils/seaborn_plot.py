"""
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Lee los archivos de texto con los valores
df1 = pd.read_csv('out26.txt', sep=' ')

#df2 = pd.read_csv('ruta/a/tu/archivo2.txt', sep=' ')
#df3 = pd.read_csv('ruta/a/tu/archivo3.txt', sep=' ')
#df4 = pd.read_csv('ruta/a/tu/archivo4.txt', sep=' ')
#df5 = pd.read_csv('ruta/a/tu/archivo5.txt', sep=' ')

# Crea la gráfica de línea con Seaborn
sns.set_palette("dark")
sns.lineplot(x='test', y='test', data=df1.iloc[::20], label='archivo1')
#sns.lineplot(x='meas. each 50 ms.', y='avg. min. dist.', data=df2.iloc[::20], label='archivo2')
#sns.lineplot(x='meas. each 50 ms.', y='avg. min. dist.', data=df3.iloc[::20], label='archivo3')
#sns.lineplot(x='meas. each 50 ms.', y='avg. min. dist.', data=df4.iloc[::20], label='archivo4')
#sns.lineplot(x='meas. each 50 ms.', y='avg. min. dist.', data=df5.iloc[::20], label='archivo5')

# Cambia los estilos de las líneas a sólidos
for line in plt.gca().lines:
    line.set_linestyle("-")

# Agrega la leyenda
plt.legend()

# Muestra la gráfica
plt.show()
"""


"""
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Lista con los nombres de los archivos de texto
files = ['out26.txt']

# Leer los archivos de texto y guardarlos en un diccionario
data = {}
for file in files:
    name = file.split('.')[0]
    data[name] = pd.read_csv(file, sep=' ', header=None)

# Crear una figura y un eje
fig, ax = plt.subplots()

# Graficar cada archivo de texto en una línea distinta

for name, df in data.items():
    df = df.astype(str)
    x = df.iloc[::20, 0]
    y = df.iloc[::20, 1]
    ax.plot(x, y, label=name)

# Configurar la figura y el eje
ax.set_xlabel('cycle (ms)')
ax.set_ylabel('avg. min. dist.')
ax.legend()

# Mostrar la figura
plt.show()
"""

import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Leer los archivos de texto y extraer los valores de la columna 10 en 10
#file_names = ['rules1', 'rules2', 'rules3', 'rules4', 'rules5', 'rules6']
file_names = ['rules1', 'rules2', 'rules3', 'rules4', 'rules5', 'rules6']

#file_names = ['regulated_fuzzy_logic1', 'regulated_fuzzy_logic2','dwb1','dwb2',]
markers = ['o','X','*','s','D','>']




#df = pd.concat((pd.read_csv(f + '.txt', sep=' ') for f in file_names))
#sns.set_theme(style="darkgrid")
#sns.lineplot(y="time", x="value", data=df, errorbar='sd') #, hue="region", , style="event"

#data = pd.concat(data_frames, axis=0)


"""
data_frames = []

#rfl

for controller in ['DWB','regulated_fuzzy_logic','regulated_pure_pursuit']:
    for num in [1,2,3,4,5]:
        file_path = controller + str(num) + ".txt"
        df = pd.read_csv(file_path, sep=' ')
        df = df.iloc[::10]
        df['line'] = controller  # Add a new column indicating which line this data belongs to
        data_frames.append(df)


file_path = file_names[1] + ".txt"
df = pd.read_csv(file_path, sep=' ')
df['line'] = "rfl"  # Add a new column indicating which line this data belongs to
data_frames.append(df)


file_path = file_names[2] + ".txt"
df = pd.read_csv(file_path, sep=' ')
df['line'] = "dwb"  # Add a new column indicating which line this data belongs to
data_frames.append(df)

file_path = file_names[3] + ".txt"
df = pd.read_csv(file_path, sep=' ')
df['line'] = "dwb"  # Add a new column indicating which line this data belongs to
data_frames.append(df)


data = pd.concat(data_frames, axis=0)

sns.set_style('darkgrid')
sns.lineplot(x='time', y='value', hue='line', style='line', data=data, errorbar='sd', markers=markers)
"""






i = 0
for file_name in file_names:
    #df = pd.read_csv(file_name + '_scan.txt', sep=' ')
    df = pd.read_csv(file_name + '.txt', sep=' ')
    df_distance = pd.read_csv(file_name + '_distance.txt', sep=' ')
    #sns.lineplot(x="time", y="value", data=df, ci='sd')
    
    y = df.iloc[::10, 0]
   # print(type(y))
    if file_name == "rules5":
        y = y.add(-0.012)
        #print(y2)
    x = df_distance.iloc[::10,0]

   # x = df.iloc[::20,1]
   # y = df.iloc[::20, 0]

    # Dibujar los valores en un diagrama de líneas distintas con diferentes colores y marcadores
    sns.lineplot(x=x, y=y, label=file_name, marker=markers[i])
    i = i+1


#plt.xlabel('time (s.)')
plt.xlabel('distance (m.)')
plt.ylabel('average minimun distance (m.)')
#plt.ylabel('closest obstacle (m.)')

plt.legend()
plt.show()

