# Adding custom list
You can take as reference the gps_coordinate_list.py

1. Create .py file that will contain the class of the custom list and the class 
of the objects to fill the list

2. Create the class of the custom object with an static method to parse from string
to the custom object.

3. Create the class of the list object with an static method to parse from string
to the list of custom objects.

4. Import the new module in __init__.py

5. Add custom list class to the list_types list

6. You can use the class that you created as a type for an argument of your handler.
