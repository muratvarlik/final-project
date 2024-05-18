import requests
import time
import pandas as pd

def get_distance_matrix(locations, api_key):
    distance_matrix = []
    for i in range(len(locations)):
        row = []
        for j in range(len(locations)):
            if i == j:
                row.append(0)  # Distance from a location to itself is 0
            else:
                origin = f'{locations[i][0]},{locations[i][1]}'
                destination = f'{locations[j][0]},{locations[j][1]}'
                url = f'https://maps.googleapis.com/maps/api/distancematrix/json?origins={origin}&destinations={destination}&key={api_key}'
                response = requests.get(url).json()
                if response['status'] == 'OK' and 'distance' in response['rows'][0]['elements'][0]:
                    distance = response['rows'][0]['elements'][0]['distance']['value']
                    distance = distance/1000
                    distance = int(distance)
                    row.append(distance)
                else:
                    row.append(None)  # In case of an error or missing distance, append None
            print(f"Row:{i} Column:{j} is finished")
        distance_matrix.append(row)
        time.sleep(1)  # Wait for 1 second to avoid exceeding the query limit
    return distance_matrix


# Example usage
df = pd.read_csv("18.01/18.01.csv")

locations = df[['Enlem', 'Boylam']]
locations = locations.to_numpy().tolist()
api_key = 'insert api key here'
distance_matrix = get_distance_matrix(locations, api_key)

df = pd.DataFrame(distance_matrix)

df.to_csv('distance_matrix_18.01_new.csv')

print('Distance matrix has been saved to distance_matrix.csv')
