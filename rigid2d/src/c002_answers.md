# ME495 - C++ Library for 2D Rigid Body Motion Q&A
Below is my response to some questions related to the development of this project:
1. **Q: What is the difference between a class and a struct in C++?**
    -  A: Class can hav private and protected inheritance and access modes, while struct only has public
inheritance and access modes. Hence, class is typically used for grouping data. Also, class pass-by-reference and class objects are stored on heap, while struct is 
pass-by-copy and struct objects are stored on stack. 

2. **Q: Why is Vector2D a struct and Transform2DClass? Refer to specific C++ Core guidelines (Classes and Class Hierarchies) in your answer. (You should refer to at least 2 guidelines).**
    -  A: First Because in a 2d vector, the two variables are independent from each other. However, in a 2D transformation matrix, 
    not all elements in the matrix free variables, because they are defined by theta, and a 2D vector. Therefore, a 2D transformation matrix has invariants. 
   Second, the transformation does not have to be public,as the public interface for the Transform2D class is sufficient for the implementation
   
3. **Q: Why are some of the constructors in Transform2D explicit? Refer to specific C++ Core guidelines (Constructors, Assignments, and Destructors) in your answer**
    -  A: To avoid implicit conversion. Implicit conversion happens when a type needs to be converted. Implicit conversions, e.g, from Vector2D to Transform2D, does not 
    make much sense (Transform2D a = Vector2D). However, we need the explicit transformation to get a transformation from pure translation.   
    
4. **Q: We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):**
        
        a. Propose three different designs for implementing the normalize functionality
        b. Discuss the pros and cons of each proposed method, in light of The C++ Core guidelines (Classes and Class Hierarchies)
        c. Which of the methods would you implement and why?
        
    -  A: 
    
            a. Method 1: Define a function called get_normalized_vec, which returns a Vector2D object
               Method 2: Add another two members: x_normalized, y_normalized and one function that normalizes the vector and stores the x, y values in 
               those variables. 
               Method 3: Define a function called normalize_vec, which will modify the Vector2D object. 
            
            b. Method 1: Pros: light weight, simple structure. 
                         Cons: once the original vector's values are updated, the normalized vector is not updated. 
                         Also, the two values can be changed manually as they are inside a structure. 
               Method 2: Pros: simple structure. 
                         Cons: No automatic updates on the normalized vector once the original vector is modified; Also the normalized 
                         vector can be modified from the outside. 
               Method 3: Pro: Light Weight
                         Cons: the original vector is modified once normalization takes place. 
               
            c. I will implement method 3, because most likely we do not need to preserve the original 2D vector.      

5. **Q: Implement the normalize functionality using the method you chose.**
    -  A:
    ``` 
      /// \brief A 2-Dimensional Vector
        struct Vector2D
        {
            double x,y;
            Vector2D(double x = 0, double y = 0):x(x), y(y){}
            /// \brief Normalize this vector
            void normalize_vec(){
                x = x/(x*x+y*y);
                y = y/(x*x+y*y);
            }
        };
    ```

6. **Q: Why is Transform2D::inv() declared const while Transform2D::operator\*\=() is not?
        Refer to C++ Core Guidelines (Constants and Immutability) in your answer**
        
     - A: This complies with the constant rule in C++. To increase readablity, we promise "no change" to the class object by implementing inv() as a constant member function. 
