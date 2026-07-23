# inv kine soln to 5 bar parllel scara
import math

# mm
X = 0
Y = 200

# link lengths 
C = 98.192
D = 110

def inv_kine(x,y):
    q_a = math.acos((x*x + y*y + C*C - D*D)/(2*C*math.sqrt(x*x + y*y))) + math.atan(x/y)
    q_b = math.acos((x*x + y*y + C*C - D*D)/(2*C*math.sqrt(x*x + y*y))) - math.atan(x/y)

    return math.degrees(q_a), math.degrees(q_b)

def main():
    # test 1
    print(inv_kine(X,Y))


if __name__ == "__main__":
    main()