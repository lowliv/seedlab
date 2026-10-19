import numpy as np

# opens the data file
with open('datafile.txt', 'r') as f:
    b = eval(f.read())

# prints the max value
print("Maximum:", max(b))

# prints the min value
print("Minimum:", min(b))

# prints the index of the value 38
print("Index of 38:", b.index(38))

# prints the max occurences
max_occurrences = max(map(b.count, b))
most_repeated = list(set([x for x in b if b.count(x) == max_occurrences]))
print("Most repeated number(s):", most_repeated, "with count:", max_occurrences)

# sorts and prints the sorted data
sorted_array = np.sort(np.array(b))
print("Sorted list:", sorted_array)

# prints even values of the sorted data
even_numbers = [x for x in sorted_array if x % 2 == 0]
print("Even numbers:", even_numbers)



# each state defined below
def state_0(char): # initial state
    if char == 'a': return state_a
    return state_0
def state_a(char): # state a
    if char == 'a': return state_a
    if char == 'b': return state_b
    return state_0
def state_b(char): # state b
    if char == 'a': return state_a
    if char == 'c': return state_c
    return state_0
def state_c(char): # state c
    if char == 'a': return state_a
    if char == 'd': return state_d
    return state_0
def state_d(char): # state d
    return state_d

# main program defined below
user_string = input("Enter a string: ") # gets user input
current_state = state_0

for character in user_string: # loops through input given defined states
    current_state = current_state(character)
    if current_state == state_d:
        print("abcd is contained in the string")
        break
