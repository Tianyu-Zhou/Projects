def describe_pet(animal_type, pet_name='dog'):
    """xian shi dong wu de xin xi"""
    print("\nI have a " + animal_type + ".")
    print("My " + animal_type + "'s name is " + pet_name.title() + ".")

describe_pet('hamster')
describe_pet(pet_name='harry', animal_type='hamster')
