import matplotlib.pyplot as plt

# Define the coordinates of the dots
base_dot = [0, 0]
first_end = [0, 1]
second_end = [0, 2]
tcp_width = 0.05
tcp_height = 1
tcp = [0, 3]
tcpx_line = [tcp[0] - tcp_width, tcp[0] + tcp_width] 
tcpy_line = [tcp[1] -tcp_height , tcp[1] ] 

dots_x = [base_dot[0], first_end[0], second_end[0], tcp[0]]
dots_y = [base_dot[1], first_end[1], second_end[1], tcp[1]-tcp_height]

# Plot the dots
plt.plot(dots_x, dots_y, 'bo')  # 'bo' means blue dots

# Connect the dots with lines
plt.plot(dots_x, dots_y, 'b-')  # 'b-' means blue lines

#Vertical TCP lines
plt.plot([tcpx_line[0], tcpx_line[0]], [tcpy_line[0], tcpy_line[1]], 'b-')
plt.plot([tcpx_line[1], tcpx_line[1]], [tcpy_line[0], tcpy_line[1]], 'b-')

#Horizontal TCP line
plt.plot([tcpx_line[0], tcpx_line[1]], [tcpy_line[0], tcpy_line[0]], 'b-')

#plot the base
plt.plot(base_dot[0], base_dot[1], 'ro')

#plot the TCP
plt.plot(tcp[0], tcp[1], 'go')

# Set the x and y axis limits
plt.xlim(-3.5, 3.5)
plt.ylim(-3.5, 3.5)

# Display the plot
plt.show()
