#ifndef __HELPERS_HPP__
#define __HELPERS_HPP__

#include <string>
#include <iostream>
#include <chrono> // TEST


/**
 * Print a string to the screen.
 * 
 * Params:
 * 		text - text to print
 * 		newlines - number of newlines to print at the end of input string
 * 		flush - whether to flush text. Generally, only needed when
 * 			printing a large body of text with manual newlines
 */
void printText(std::string text, int newlines, bool flush)
{
    // TODO: Consider adding param for width of text line
    std::cout << text;

    for (int i = 0; i < newlines; i++) {
        if (i > 1) {
            std::cout << "\n";
        }
        else {
            std::cout << std::endl;
        }
    }

    if (flush) {
        std::cout.flush();
    }

    return;
}


// TEST
auto start = std::chrono::high_resolution_clock::now();
void startTiming()
{
    start = std::chrono::high_resolution_clock::now();
}

void stopTiming()
{
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Time: " << duration.count() << std::endl;
}

#endif // __HELPERS_HPP__