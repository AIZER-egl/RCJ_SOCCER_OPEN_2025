#ifndef PICO_LIB_USB_H
#define PICO_LIB_USB_H

#include <iostream>
#include <string>

#define USB_ENDL "\r\n"

class USB {
public:
    class Printer {
    public:
        template <typename T>
        Printer& operator<<(const T& data) {
            std::cout << data;
            return *this;
        }

        Printer& operator<<(std::ostream& (*manipulator)(std::ostream&)) {
            std::cout << manipulator;
            return *this;
        }
    };

    static Printer print;

    unsigned long read() {
        // WARNING: This will interrupt the main thread, use with caution.
        // In test local project there is a version that does not interrupt the main thread.
        // If problems arise, use the version that does not interrupt the main thread.
        unsigned long data;
        std::cin >> data;
        return data;
    }
};

#endif // PICO_LIB_USB_H
