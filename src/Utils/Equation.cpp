#define BIORBD_API_EXPORTS
#include "Utils/Equation.h"

#include <boost/lexical_cast.hpp>
#include "Utils/Error.h"

std::vector<biorbd::utils::Equation> biorbd::utils::Equation::prepareMathSymbols(){
    // Classés en ordre de priorité des opérations
    std::vector<biorbd::utils::Equation> symbols;
    symbols.push_back("(");
    symbols.push_back(")");
    symbols.push_back("e");
    symbols.push_back("/");
    symbols.push_back("*");
    symbols.push_back("+");
    symbols.push_back("-");

    return symbols;
}

biorbd::utils::Equation::Equation() :
    biorbd::utils::String("")
{

}

biorbd::utils::Equation::Equation(const char *c) :
    biorbd::utils::String(c)
{

}

biorbd::utils::Equation::Equation(const biorbd::utils::String &s) :
    biorbd::utils::String(s)
{

}

biorbd::utils::Equation::Equation(const std::basic_string<char> &c) :
    biorbd::utils::String(c)
{

}

std::vector<biorbd::utils::Equation> biorbd::utils::Equation::splitIntoEquation(biorbd::utils::Equation wholeEq, const std::map<biorbd::utils::Equation, double>& variables){
    // variable de sortie
    std::vector<biorbd::utils::Equation> eq;

    // Déclaration des marqueurs arithmétiques
    const std::vector<biorbd::utils::Equation>& symbols(prepareMathSymbols());

    // Sur tout le long du string
    while (1){
        int firstIdx(static_cast<int>(wholeEq.size())+1);// Mettre un index trop grand
        unsigned int toStop(0); // Mettre stop a 0
        // Balayer chaque symbols pour voir s'ils se trouvent dans l'equation
        for (unsigned int i=0; i<symbols.size(); ++i){
            int idx = static_cast<int>(wholeEq.find(symbols[i]));
            if (idx < 0){ // Si non, le noter
                ++toStop;
                continue;
            }
            if (idx < firstIdx && idx != -1) // si oui, regarder s'il est avant un autre symbol
                firstIdx = idx;
        }
        if (toStop == symbols.size()){ // S'il ne reste aucun symbole, sortir du while
            eq.push_back(wholeEq);
            break;
        }
        if (wholeEq.size() == 1){
            eq.push_back(wholeEq(0));
            break;
        }
        if (firstIdx+1 == static_cast<int>(wholeEq.size())){
            eq.push_back(wholeEq.substr(0,static_cast<unsigned int>(firstIdx)));
            eq.push_back(wholeEq(static_cast<unsigned int>(firstIdx)));
            break;
        }
        else if (firstIdx == 0){
            if (!wholeEq(0).compare("-")){
                // Si l'équation commencer par un -
                std::vector<biorbd::utils::Equation> tp(splitIntoEquation(wholeEq.substr(1), variables));
                tp[0] = "-" + tp[0];
                if (eq.size()==0) // S'il n'y a rien avant, est faux dans les cas 1e-x
                    return tp;
                else{
                    eq.push_back(wholeEq(0));
                    wholeEq = wholeEq.substr(1);
                }
            }
            else if (!wholeEq(0).compare("+")){
                // Si l'équation commencer par un +
                std::vector<biorbd::utils::Equation> tp(splitIntoEquation(wholeEq.substr(1), variables));
                tp[0] = "+" + tp[0];
                if (eq.size()==0) // S'il n'y a rien avant, est faux dans les cas 1e-x
                    return tp;
                else{
                    eq.push_back(wholeEq(0));
                    wholeEq = wholeEq.substr(1);
                }
            }
            else{
                eq.push_back(wholeEq(0));
                wholeEq = wholeEq.substr(1);
            }
        }
        else{
            eq.push_back(wholeEq.substr(0,static_cast<unsigned int>(firstIdx))); // Prendre tout ce qui vient avant le symbole
            eq.push_back(wholeEq(static_cast<unsigned int>(firstIdx))); // Récupérer le symbole
            wholeEq = wholeEq.substr(static_cast<unsigned int>(firstIdx)+1); // Garder tout ce qu'il y a apres le symbole et recommencer
        }

    }

    // Remplacer les constantes par un nombre
    replaceCste(eq);

    // Remplacer les variables par un nombre
    replaceVar(eq, variables);

    // Retourner l'équation
    return eq;
}

void biorbd::utils::Equation::replaceCste(std::vector<biorbd::utils::Equation> &eq){
    for (unsigned int i=0; i<eq.size(); ++i)
        if (!eq[i].tolower().compare("pi"))
            eq[i] = boost::lexical_cast< std::string>(PI);
}

void biorbd::utils::Equation::replaceVar(
        std::vector<biorbd::utils::Equation> &eq,
        const std::map<biorbd::utils::Equation, double>&var){
    for (unsigned int i=0; i<eq.size(); ++i)
        if (var.find(eq[i]) == var.end())
            continue;
        else
            eq[i] = boost::lexical_cast< std::string>( var.find(eq[i])->second );
}


double biorbd::utils::Equation::resolveEquation(std::vector<biorbd::utils::Equation> eq){
    return resolveEquation(eq,0);
}

double biorbd::utils::Equation::resolveEquation(
        std::vector<biorbd::utils::Equation> eq,
        unsigned int math){
    // Si on a traité tout
    if (eq.size() == 1)
        return boost::lexical_cast<double>(eq[0]);

    // Déclaration des marqueurs arithmétiques
    const std::vector<biorbd::utils::Equation>& symbols(prepareMathSymbols());
    std::vector<biorbd::utils::Equation> eq2;
    bool continuer(true);

    for (unsigned int j=0; j<eq.size(); ++j){
        if (!eq[j].compare(symbols[math]) && continuer){
            if (j==0 && (!symbols[math].compare("+") || !symbols[math].compare("-"))){
                // Écraser la valeur précédente
                if (!symbols[math].compare("+"))
                    eq2[j-1] = boost::lexical_cast<std::string>(0.0 + boost::lexical_cast<double>(eq[j+1]));
                else if (!symbols[math].compare("-"))
                    eq2[j-1] = boost::lexical_cast<std::string>(0.0 - boost::lexical_cast<double>(eq[j+1]));
            }
            else{
                // Écraser la valeur précédente
                if (!symbols[math].compare("(")){
                    std::vector<biorbd::utils::Equation> eq_tp;
                    bool foundIdx(false);
                    int cmpValues(0);
                    unsigned int cmpOpen(0);
                    for (unsigned int k=j+1; k<eq.size();++k){
                        if (!eq[k].compare("("))
                            cmpOpen++;
                        else if (!eq[k].compare(")")){
                            if (cmpOpen == 0){
                                foundIdx = true;
                                break;
                            }
                            else
                                cmpOpen--;
                        }

                        eq_tp.push_back(eq[k]);
                        ++cmpValues;
                    }
                    biorbd::utils::Error::check(foundIdx, "You must close brackets!");

                    eq2.push_back(boost::lexical_cast<std::string>(boost::lexical_cast<std::string>(resolveEquation(eq_tp))));
                    j+=static_cast<unsigned int>(cmpValues);
                }
                else if (!symbols[math].compare("/"))
                    eq2[j-1] = boost::lexical_cast<std::string>(boost::lexical_cast<double>(eq[j-1]) / boost::lexical_cast<double>(eq[j+1]));
                else if (!symbols[math].compare("*"))
                    eq2[j-1] = boost::lexical_cast<std::string>(boost::lexical_cast<double>(eq[j-1]) * boost::lexical_cast<double>(eq[j+1]));
                else if (!symbols[math].compare("+"))
                    eq2[j-1] = boost::lexical_cast<std::string>(boost::lexical_cast<double>(eq[j-1]) + boost::lexical_cast<double>(eq[j+1]));
                else if (!symbols[math].compare("-"))
                    eq2[j-1] = boost::lexical_cast<std::string>(boost::lexical_cast<double>(eq[j-1]) - boost::lexical_cast<double>(eq[j+1]));
                else if (!symbols[math].compare("e")){
                    if (!eq[j+1].compare("-")){
                        eq2[j-1] = boost::lexical_cast<std::string>(boost::lexical_cast<double>(eq[j-1]) * pow(10,-1*boost::lexical_cast<double>(eq[j+2])));
                        ++j;
                    }
                    else if (!eq[j+1].compare("+")){
                        eq2[j-1] = boost::lexical_cast<std::string>(boost::lexical_cast<double>(eq[j-1]) * pow(10,boost::lexical_cast<double>(eq[j+2])));
                        ++j;
                    }
                    else
                        eq2[j-1] = boost::lexical_cast<std::string>(boost::lexical_cast<double>(eq[j-1]) * pow(10,boost::lexical_cast<double>(eq[j+1])));

                }
            }

            j+=2;
            continuer = false;
        }
        if (j<eq.size())
            eq2.push_back(eq[j]);
    }

    if (continuer)
        return resolveEquation(eq2, ++math);
    else
        return resolveEquation(eq2, math);
}

double biorbd::utils::Equation::resolveEquation(biorbd::utils::Equation wholeEq, const std::map<biorbd::utils::Equation, double>& variables){
    return resolveEquation(splitIntoEquation(wholeEq, variables));
}
double biorbd::utils::Equation::resolveEquation(biorbd::utils::Equation wholeEq){
    std::map<biorbd::utils::Equation, double> dumb;
    return resolveEquation(splitIntoEquation(wholeEq, dumb));
}
