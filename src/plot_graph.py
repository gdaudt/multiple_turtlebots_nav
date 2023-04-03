import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame
df = pd.read_csv('success.csv')

# Set the 'agents' column as the index
df.set_index('agents', inplace=True)

# Convert the success rate columns to floats (remove the % sign and divide by 100)
df['k2 success rate'] = df['k2 success rate'].str.rstrip('%').astype('float') / 100
df['k3 success rate'] = df['k3 success rate'].str.rstrip('%').astype('float') / 100
df['CCBS success rate'] = df['CCBS success rate'].str.rstrip('%').astype('float') / 100

# Create a line plot of the success rates
plt.plot(df.index, df['k2 success rate'], label='k2')
plt.plot(df.index, df['k3 success rate'], label='k3')
plt.plot(df.index, df['CCBS success rate'], label='CCBS')

# Add labels and title to the plot
plt.xlabel('Number of Agents')
plt.ylabel('Success Rate')
plt.title('Success Rate by Number of Agents')

# Add a legend to the plot
plt.legend()

# Display the plot
plt.show()