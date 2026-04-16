import pandas as pd


input_file = 'ProcessedDiLTracks\Monaco_Traj_Mat.xlsx'
input_name = input_file.split('\\')[-1].split('.')[0]
output_file = f'{input_file}_v2.xlsx'

df = pd.read_excel(input_file)

df_filtered = df.drop(df.index[2::2])

df_filtered.to_excel(output_file, index=False)

print(f"Filtered data has been saved to {output_file}")