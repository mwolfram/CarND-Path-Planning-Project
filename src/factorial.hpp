// for testing the unit test framework

#ifndef FACTORIAL_HPP
#define FACTORIAL_HPP

unsigned int Factorial( unsigned int number ) {
    if (number == 0) {
        return 1;
    }
    return number <= 1 ? number : Factorial(number-1)*number;
}

#endif
