def calculate_gps_from_xy(x, y, origin_lat, origin_lon, scale_lat_per_unit_x, scale_lon_per_unit_y):
    """
    Calcula as coordenadas GPS (latitude e longitude) a partir das coordenadas (X, Y).
    
    Args:
        x (float): Coordenada X no ambiente simulado.
        y (float): Coordenada Y no ambiente simulado.
        origin_lat (float): Latitude de origem (coordenada (0,0)).
        origin_lon (float): Longitude de origem (coordenada (0,0)).
        scale_lat_per_unit_x (float): Fator de escala para latitude por unidade de X.
        scale_lon_per_unit_y (float): Fator de escala para longitude por unidade de Y.
        
    Returns:
        tuple: Coordenadas GPS (latitude, longitude) calculadas.
    """
    lat = origin_lat + (x * scale_lat_per_unit_x)
    lon = origin_lon + (y * scale_lon_per_unit_y)
    
    return lat, lon

# Parâmetros conhecidos
origin_lat = -22.4151892013  # Latitude do ponto (0, 0, 0)
origin_lon = -45.447777211   # Longitude do ponto (0, 0, 0)

# Fatores de escala (calculados anteriormente ou conhecidos)
scale_lat_per_unit_x = -9.211474e-06  # Mudança de latitude por unidade de X
scale_lon_per_unit_y = -9.676528e-06  # Mudança de longitude por unidade de Y

# Novo ponto (X, Y) no ambiente simulado
x = 270.0076
y =  312.1785
# Cálculo das coordenadas GPS
new_lat, new_lon = calculate_gps_from_xy(x, y, origin_lat, origin_lon, scale_lat_per_unit_x, scale_lon_per_unit_y)
print(f"Coordenadas GPS para o ponto ({x}, {y}): Latitude = {new_lat}, Longitude = {new_lon}")
