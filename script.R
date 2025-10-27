# The purpose of this is to understand, PID contoller, understand how R reads CSV, Im new in R
print("Script started")
system("gcc main.c -o main -lm") 
system("./main") 

data <- read.csv("data.csv")

data$Delta <- as.numeric(data$Delta)

data$Setpoint <- as.numeric(data$Setpoint)
data$Output <- as.numeric(data$Output)
data$Plant <- as.numeric(data$Plant)

data$P <- as.numeric(data$P)
data$I <- as.numeric(data$I)
data$D <- as.numeric(data$D)

# Plot 1
plot(data$Sim, type="l", col="blue",
     xlab="Over time", ylab="Value", main="Sensor simulation vs Plant (system that is being controlled)")

lines(data$Plant, col="red", lwd=2)

legend("bottomleft", legend=c("Measurement", "Plant (Sensor)"), 
       col=c("blue", "red"), lty=c(1,1), lwd=c(1,2))

# Plot 2
#  min and max values across both datasets to set the Y-axis limit
y_range <- range(c(data$Output, data$Delta))


plot(data$Output, type = "l", col = "blue", lwd = 2,
     xlab = "Iteration/Time", 
     ylab = "Value", 
     main = "Comparison of Output and Error",
     ylim = y_range 
)


lines(data$Delta, col = "red", lwd = 2, lty = 2)
lines(data$Setpoint, col = "green", lwd = 2, lty = 2)

# print(data$Delta)
# print(data$Output)

legend("topright", 
       legend = c("Output", "Error", "Target"), 
       col = c("blue", "red", "green"), 
       lty = c(1, 2, 3),
       lwd = 2)         

# Plot 3 PID plot
y_range <- range(c(data$P, data$I, data$D))

plot(data$P, type = "l", col = "orange", lwd = 2,
     xlab = "Iteration/Time", 
     ylab = "Value", 
     main = "PID term",
     ylim = y_range 
)

lines(data$I, col = "purple", lwd = 2, lty = 2)
lines(data$D, col = "green", lwd = 2, lty = 2)
lines(data$Delta, col = "red", lwd = 2, lty = 2)

legend("topright", 
       legend = c("P", "I", "D", "Error"), 
       col = c("orange", "purple", "green", "red"), 
       lty = c(1, 2, 3, 4),
       lwd = 2)         



print("Script end!")

