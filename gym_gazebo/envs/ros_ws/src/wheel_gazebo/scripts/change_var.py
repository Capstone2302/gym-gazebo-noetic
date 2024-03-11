def main():
    # Define some variables
    while True:
        ki = 0
        kd = 0
        kp = 0

        # Get user input
        user_input = input("enter var name: ")

        # Check user input and modify variables accordingly
        if user_input == 'ki':
            new_value = int(input("Enter the new value for ki: "))
            ki = new_value
        elif user_input == 'kp':
            new_value = int(input("Enter the new value for kp: "))
            kp = new_value
        elif user_input == 'kp':
            new_value = int(input("Enter the new value for kd: "))
            kd = kd
        else:
            print("Invalid input")

        # Print updated variables
        print("Updated kp:", kp)
        print("Updated ki:", ki)
        print("Updated kd:", kd)

if __name__ == "__main__":
    main()