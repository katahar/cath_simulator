import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
file_path = 'Traced_anatomical_model.csv'
data = pd.read_csv(file_path)

# Print column names
print("Column Names:", data.columns)

# Plot the points
plt.scatter(data['X'], data['Y'])
plt.title('Traced Anatomical Model')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
