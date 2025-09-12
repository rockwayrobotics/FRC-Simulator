input_string = input("Enter a verb in the infinitive form: ")
erverbs = ("er, Er, ER, eR")

if input_string.endswith(suffix_to_check):
    print(f"The string ends with '{suffix_to_check}'.")
else:
    print(f"The string does not end with '{suffix_to_check}'.")