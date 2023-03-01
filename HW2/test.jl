# using PlotlyJS
# p1 = scatter(x=1:3, y=4:6 ,xlabel="X",ylabel="Y")

# relayout!(p, title_text="Side by side layout (1 x 2)")
# p
# fig = make_subplots(rows=3, cols=1, shared_xaxes=true, vertical_spacing=0.02, x_title="time(s)")
# tvec = collect(1:10)
# ydata = collect(1:10)
# add_trace!(fig, scatter(x=tvec, y=ydata,
#             line=attr(color="rgba(0,100,80,1)"),
#             name="true", ylabel="x"), row=1, col=1)
# add_trace!(fig, scatter(x=tvec, y=ydata,
#             line=attr(color="rgba(10,10,200,1)"),
#             showlegend=true, ylabel="x"), row=2, col=1)
# add_trace!(fig, scatter(x=tvec, y=ydata,
#             line=attr(color="rgba(70,10,100,1)"),
#             showlegend=false, ylabel="x"), row=3, col=1)
# display(fig)


using Plots

# generate some example data
x = 1:10
y1 = rand(10)
y2 = rand(10) * 10
y3 = rand(10) * 100

# create the subplots with different y labels
p1 = plot(x, y1, xlabel = "Time", ylabel="Label 1")
p2 = plot(x, y2, xlabel = "Time", ylabel="Label 2")
p3 = plot(x, y3, xlabel = "Time", ylabel="Label 3")

# arrange the subplots in a grid
plot(p1, p2, p3, p1, layout=(2,2))

# plot!(p2, p2, p3, layout=(1,3))

# using PlotlyJS
#
# # generate some example data
# x = 1:10
# y1 = rand(10)
# y2 = rand(10) * 10
# y3 = rand(10) * 100
#
# # create the subplots with different y labels
# trace1 = scatter(x=x, y=y1, yaxis="y1", name="Label 1")
# trace2 = scatter(x=x, y=y2, yaxis="y2", name="Label 2")
# trace3 = scatter(x=x, y=y3, yaxis="y3", name="Label 3")
#
# # arrange the subplots in a grid
# layout = Layout(grid=attr(rows=3, columns=1),
#                 yaxis1=attr(title="Label 1"),
#                 yaxis2=attr(title="Label 2"),
#                 yaxis3=attr(title="Label 3"))
#
# subplot = Subplots([trace1, trace2, trace3], layout)
#
# plot([subplot])

#
# using PlotlyJS
#
# # generate some example data
# x = 1:10
# y1 = rand(10)
# y2 = rand(10) * 10
# y3 = rand(10) * 100
#
# # create the subplots with different y labels
# trace1 = scatter(x=x, y=y1, name="Label 1")
# trace2 = scatter(x=x, y=y2, name="Label 2")
# trace3 = scatter(x=x, y=y3, name="Label 3")
#
# # arrange the subplots in a grid
# fig = subplots([trace1, trace2, trace3], layout_grid=attr(rows=3, columns=1))
#
# # set the y-axis label for each subplot
# fig.layout[:yaxis1][:title] = "Label 1"
# fig.layout[:yaxis2][:title] = "Label 2"
# fig.layout[:yaxis3][:title] = "Label 3"
#
# plot(fig)


# using PlotlyJS
#
# # generate some example data
# x = 1:10
# y1 = rand(10)
# y2 = rand(10) * 10
# y3 = rand(10) * 100
#
# # create the subplots with different y labels
# trace1 = scatter(x=x, y=y1, name="Label 1")
# trace2 = scatter(x=x, y=y2, name="Label 2")
# trace3 = scatter(x=x, y=y3, name="Label 3")
#
# # arrange the subplots in a grid
# fig = plot([trace1, trace2, trace3], layout=attr(grid=attr(rows=3, columns=1)))
#
# # set the y-axis label for each subplot
# fig.layout[:yaxis1][:title] = "Label 1"
# fig.layout[:yaxis2][:title] = "Label 2"
# fig.layout[:yaxis3][:title] = "Label 3"
#
# plot(fig)

#
# using PlotlyJS
#
# # generate some example data
# x = 1:10
# y1 = rand(10)
# y2 = rand(10) * 10
# y3 = rand(10) * 100
#
# # create the subplots with different y labels
# trace1 = scatter(x=x, y=y1, name="Label 1")
# trace2 = scatter(x=x, y=y2, name="Label 2")
# trace3 = scatter(x=x, y=y3, name="Label 3")
#
# # arrange the subplots in a grid
# fig = plot([trace1, trace2, trace3], layout=attr(grid=attr(rows=3, columns=1)))
#
# # set the y-axis label for each subplot
# fig.layout[:yaxis1][:title] = "Label 1"
# fig.layout[:yaxis2][:title] = "Label 2"
# fig.layout[:yaxis3][:title] = "Label 3"
#
# # create an animation
# frames = [attr(data=[scatter(x=x, y=rand(10)) for i in 1:3])] # replace with your own animation frames
# ani = Plotly.animate(fig, frames)
#
# # display the animation
# plot(ani)


# using PlotlyJS
#
# # Generate example data
# x1 = 1:10; y1 = rand(10)
# x2 = 2:2:20; y2 = rand(10) * 10
# x3 = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]; y3 = rand(10) * 100
#
# # Define subplots
# trace1 = scatter(x=x1, y=y1, mode="lines+markers", name="Label 1")
# trace2 = scatter(x=x2, y=y2, mode="lines+markers", name="Label 2")
# trace3 = scatter(x=x3, y=y3, mode="lines+markers", name="Label 3")
#
# subplot_titles = ["Subplot 1", "Subplot 2", "Subplot 3"]
# xaxis_labels = ["X Label 1", "X Label 2", "X Label 3"]
# yaxis_labels = ["Y Label 1", "Y Label 2", "Y Label 3"]
#
# fig = PlotlyJS.plot([trace1, trace2, trace3],
#     layout=attr(
#         grid=attr(rows=3, columns=1),
#         subplot_titles=subplot_titles,
#         xaxis1=attr(title=xaxis_labels[1]),  # Set x-axis label for subplot 1
#         xaxis2=attr(title=xaxis_labels[2]),  # Set x-axis label for subplot 2
#         xaxis3=attr(title=xaxis_labels[3]),  # Set x-axis label for subplot 3
#         yaxis1=attr(title=yaxis_labels[1]),  # Set y-axis label for subplot 1
#         yaxis2=attr(title=yaxis_labels[2]),  # Set y-axis label for subplot 2
#         yaxis3=attr(title=yaxis_labels[3])   # Set y-axis label for subplot 3
#     )
# )
#
# PlotlyJS.plot(fig)

using Plots

# Define x, y, z data
x = rand(10)
y = rand(10)
z = rand(10)

# Create a 3D scatter plot
scatter(x, y, z, markersize=5, markerstrokewidth=0, color=:red, legend=false)

# Set the axis labels and title
xlabel!("X Axis")
ylabel!("Y Axis")
zlabel!("Z Axis")
title!("3D Scatter Plot")

using Plots

# Define x, y, and z data for two lines
x1 = 0:0.1:1
y1 = sin.(2π*x1)
z1 = abs.(cos.(2π*x1))

x2 = 0:0.1:1
y2 = cos.(2π*x2)
z2 = sin.(2π*x2)

# Create a 3D line plot
p = plot3d([x1], [y1], [z1], line=(:red, 2), legend=false)
# Set the axis labels and title
xlabel!("X Axis")
ylabel!("Y Axis")
zlabel!("Z Axis")
title!("3D Line Plot")
