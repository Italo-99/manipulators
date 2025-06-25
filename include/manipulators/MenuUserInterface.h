#ifndef MENU_USER_INTERFACE_H
#define MENU_USER_INTERFACE_H

#include <iostream>
#include <memory>
#include <vector>
#include <tuple>
#include <functional> 
#include <cmath>
#include <sstream>
#include <iomanip>

/*
 ====== MenuUserInterface class ======
 This is a template class to create a user interface through the 
 terminal for any node. 
 To implement this class you can add a MenuUserInterface object as
 a member variable of your class, then you can initialize it with 
 a shared pointer to the class itself, this must be done only after
 the class has been initialized, like inside the spinner.
 Remember to add the following line to explicitly declare the template
 at the end of your node file:

    template class MenuUserInterface<YourNodeClass>;

 You can see an example of usage in the ManipulatorMenu node.
*/

namespace MenuUserInterfaceUtils
{
    enum PaddingType
    {
        LEFT,
        RIGHT,
        CENTER
    };

    // Utility function to apply padding to a title string
    static std::string applyPadding(std::string title, 
                                    const unsigned long size = 50, 
                                    const char padding_char = '=', 
                                    PaddingType type = CENTER)
    {
        if (title.size() < size)
        {
            switch (type)
            {
                case RIGHT:
                    return title + std::string(size - title.size(), padding_char);
                case LEFT:
                    return std::string(size - title.size(), padding_char) + title;
                case CENTER: {
                        int padding = floor((size - title.size()) / 2);
                        return std::string(padding, padding_char) + " " + title + " " + std::string(padding, padding_char);
                    }
                default:
                    return title;
            }
        }
        else
        {
            return title;
        }
    }

    [[maybe_unused]]
    static std::string formatDouble(double value,
                                    int precision    = 2,
                                    int width        = 5,
                                    PaddingType type = RIGHT,
                                    char padding_char = ' ')
    {

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        std::string num = oss.str();          // e.g. "12.35"

        if (static_cast<int>(num.size()) >= width)
            return num;                       // no truncation

        switch (type)
        {
            case RIGHT:
                return num + std::string(width - num.size(), padding_char);

            case LEFT:
                return std::string(width - num.size(), padding_char) + num;

            case CENTER:
            default:
                int pad_left  = (width - num.size()) / 2;
                int pad_right = width - num.size() - pad_left;
                return std::string(pad_left,  padding_char) +
                    num +
                    std::string(pad_right, padding_char);
        }
    }

}

template <typename SuperClass>
class MenuUserInterface
{
public:

    MenuUserInterface(SuperClass *super)
        : super_(super) {}

    void printMenu();                   //Print a list of every choice divided in the appropriate sections
    int getUserChoice();                //Get the user choice through cli
    void processChoice(int choice);   

    void addChoice(int id, std::string description, void (SuperClass::*function)(void));
    void addChoice(std::string description, void (SuperClass::*function)(void));

    //Make a section of the menu, its title will be printed at the top
    //section_start and section_end are both inclusive
    void addSection(std::string section_name, int section_start, int section_end);

    int len_ = 0;       //Number of choices
    int last_ = -1;     //Choice with the highest number

private:
    std::vector<std::tuple<int, std::string, void (SuperClass::*)(void)>> choices_;
    std::vector<std::tuple<std::string, int, int>> sections_;
    SuperClass *super_;

    std::tuple<int, std::string, void (SuperClass::*)(void)> getChoice(int id); //Get choice by id
};

// Function Definitions

template <typename SuperClass>
void MenuUserInterface<SuperClass>::printMenu()
{
    for (auto section : sections_)
    {
        std::cout << std::endl << MenuUserInterfaceUtils::applyPadding(std::get<0>(section)) << std::endl;
        for (int i = std::get<1>(section); i <= std::get<2>(section); i++)
        {
            auto choice = getChoice(i);
            printf("%d. - %s\n", std::get<0>(choice), std::get<1>(choice).c_str());
        }
    }
}

template <typename SuperClass>
int MenuUserInterface<SuperClass>::getUserChoice()
{
    int choice;
    std::cout << "Enter your choice: ";
    std::cin >> choice;
    return choice;
}

template <typename SuperClass>
void MenuUserInterface<SuperClass>::processChoice(int choice)
{
    auto choice_tuple = getChoice(choice);
    if (std::get<0>(choice_tuple) == -1)
    {
        std::cout << "Invalid choice" << std::endl;
        return;
    }
    auto function = std::get<2>(choice_tuple);
    std::bind(function, super_)();
}

template <typename SuperClass>
void MenuUserInterface<SuperClass>::addChoice(int id, std::string description, void (SuperClass::*function)(void))
{
    len_++;
    if (id > last_)
    {
        last_ = id;
    }
    choices_.push_back(std::make_tuple(id, description, function));
}

template <typename SuperClass>
void MenuUserInterface<SuperClass>::addChoice(std::string description, void (SuperClass::*function)(void))
{
    // If no id is provided, add the choice at last_ + 1 index
    addChoice(last_ + 1, description, function);
}

template <typename SuperClass>
void MenuUserInterface<SuperClass>::addSection(std::string section_name, int section_start, int section_end)
{
    sections_.push_back(std::make_tuple(section_name, section_start, section_end));
}

template <typename SuperClass>
std::tuple<int, std::string, void (SuperClass::*)(void)> MenuUserInterface<SuperClass>::getChoice(int id)
{
    for (auto choice : choices_)
    {
        if (std::get<0>(choice) == id)
        {
            return choice;
        }
    }
    return std::make_tuple(-1, "Invalid choice", nullptr);
}

#endif /* MENU_USER_INTERFACE_H */
