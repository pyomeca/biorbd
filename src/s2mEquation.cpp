#define BIORBD_API_EXPORTS
#include "../include/s2mEquation.h"



std::vector<s2mEquation> s2mEquation::prepareMathSymbols(){
    // Classés en ordre de priorité des opérations
    std::vector<s2mEquation> symbols;
    symbols.push_back("(");
    symbols.push_back(")");
    symbols.push_back("e");
    symbols.push_back("/");
    symbols.push_back("*");
    symbols.push_back("+");
    symbols.push_back("-");


    return symbols;
}

s2mEquation::s2mEquation()
    : s2mString("")
{

}

s2mEquation::s2mEquation(const char *c)
    : s2mString(c)
{

}

s2mEquation::s2mEquation(const s2mString &s)
    : s2mString(s)
{

}

s2mEquation::s2mEquation(const std::basic_string<char> &c)
    : s2mString(c)
{

}

s2mEquation::~s2mEquation()
{

}

std::vector<s2mEquation> s2mEquation::splitIntoEquation(s2mEquation wholeEq, const std::map<s2mEquation, double>& variables){
    // variable de sortie
    std::vector<s2mEquation> eq;

    // Déclaration des marqueurs arithmétiques
    std::vector<s2mEquation> symbols(prepareMathSymbols());

    // Sur tout le long du string
    while (1){
        int firstIdx(wholeEq.size()+1);// Mettre un index trop grand
        unsigned int toStop(0); // Mettre stop a 0
        // Balayer chaque symbols pour voir s'ils se trouvent dans l'equation
        for (unsigned int i=0; i<symbols.size(); ++i){
            int idx = wholeEq.find(symbols[i]);
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
        if (firstIdx+1 == (int)wholeEq.size()){
            eq.push_back(wholeEq.substr(0,firstIdx));
            eq.push_back(wholeEq(firstIdx));
            break;
        }
        else if (firstIdx == 0){
            if (!wholeEq(0).compare("-")){
                // Si l'équation commencer par un -
                std::vector<s2mEquation> tp = splitIntoEquation(wholeEq.substr(1), variables);
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
                std::vector<s2mEquation> tp = splitIntoEquation(wholeEq.substr(1), variables);
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
            eq.push_back(wholeEq.substr(0,firstIdx)); // Prendre tout ce qui vient avant le symbole
            eq.push_back(wholeEq(firstIdx)); // Récupérer le symbole
            wholeEq = wholeEq.substr(firstIdx+1); // Garder tout ce qu'il y a apres le symbole et recommencer
        }

    }

    // Remplacer les constantes par un nombre
    replaceCste(eq);

    // Remplacer les variables par un nombre
    replaceVar(eq, variables);

    // Retourner l'équation
    return eq;
}

void s2mEquation::replaceCste(std::vector<s2mEquation> &eq){
    for (unsigned int i=0; i<eq.size(); ++i)
        if (!eq[i].tolower().compare("pi"))
            eq[i] = boost::lexical_cast< std::string>(PI);
}

void s2mEquation::replaceVar(std::vector<s2mEquation> &eq, const std::map<s2mEquation, double>&var){
    for (unsigned int i=0; i<eq.size(); ++i)
        if (var.find(eq[i]) == var.end())
            continue;
        else
            eq[i] = boost::lexical_cast< std::string>( var.find(eq[i])->second );
}


double s2mEquation::resolveEquation(std::vector<s2mEquation> eq){
    return resolveEquation(eq,0);
}

double s2mEquation::resolveEquation(std::vector<s2mEquation> eq, unsigned int math){
    // Si on a traité tout
    if (eq.size() == 1)
        return boost::lexical_cast<double>(eq[0]);

    // Déclaration des marqueurs arithmétiques
    std::vector<s2mEquation> symbols(prepareMathSymbols());
    std::vector<s2mEquation> eq2;
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
                    std::vector<s2mEquation> eq_tp;
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
                    s2mError::s2mAssert(foundIdx, "You must close brackets!");

                    eq2.push_back(boost::lexical_cast<std::string>(boost::lexical_cast<std::string>(resolveEquation(eq_tp))));
                    j+=cmpValues;
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

double s2mEquation::resolveEquation(s2mEquation wholeEq, const std::map<s2mEquation, double>& variables){
    return resolveEquation(splitIntoEquation(wholeEq, variables));
}
double s2mEquation::resolveEquation(s2mEquation wholeEq){
    std::map<s2mEquation, double> dumb;
    return resolveEquation(splitIntoEquation(wholeEq, dumb));
}