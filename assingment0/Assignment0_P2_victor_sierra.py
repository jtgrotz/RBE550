import turtle
turtle.color('green')
turtle.getscreen()
turtle.setheading(90)
for i in range(0,3):
    for j in range(0,3):
        turtle.forward(200)
        print("new point")
        print(turtle.xcor())
        print(turtle.ycor())
        if (abs(turtle.xcor()) > 1 or abs(turtle.ycor()) > 1):
            turtle.right(120)
turtle.exitonclick()    
      
    