#define BIORBD_API_EXPORTS
#include "ModelReader.h"

#include <limits.h>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/IfStream.h"
#include "Utils/String.h"
#include "Utils/Equation.h"
#include "Utils/Vector.h"
#include "Utils/Node3d.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/BoneMesh.h"
#include "RigidBody/BoneCharacteristics.h"
#include "RigidBody/IMU.h"
#include "RigidBody/Patch.h"
#include "RigidBody/NodeBone.h"

#ifdef MODULE_ACTUATORS
#include "Actuators/ActuatorConstant.h"
#include "Actuators/ActuatorLinear.h"
#include "Actuators/ActuatorGauss3p.h"
#include "Actuators/ActuatorGauss6p.h"
#endif // MODULE_ACTUATORS

#ifdef MODULE_MUSCLES
#include "Muscles/Muscle.h"
#include "Muscles/Geometry.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/WrappingCylinder.h"
#include "Muscles/FatigueParameters.h"
#include "Muscles/State.h"
#include "Muscles/Characteristics.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/PathChangers.h"
#endif // MODULE_MUSCLES

// ------ Public methods ------ //
biorbd::Model biorbd::Reader::readModelFile(const biorbd::utils::Path &path)
{
    // Ajouter les éléments entrés
    biorbd::Model model;
    readModelFile(path, &model);
    return model;
}

void biorbd::Reader::readModelFile(
        const biorbd::utils::Path &path,
        biorbd::Model *model)
{	// Ouverture du fichier
    if (!path.isFileReadable())
        biorbd::utils::Error::raise("File " + path.absolutePath()
                                    + " could not be open");

#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    // Lecture du fichier
    biorbd::utils::String main_tag;
    biorbd::utils::String property_tag;
    biorbd::utils::String subproperty_tag;

    // Variable utilisée pour remplacer les doubles
    std::map<biorbd::utils::Equation, double> variable;

    // Déterminer la version du fichier
    file.readSpecificTag("version", main_tag);
    unsigned int version(static_cast<unsigned int>(atoi(main_tag.c_str())));
    biorbd::utils::Error::check((version == 1 || version == 2 || version == 3 || version == 4),
                                "Version " + main_tag + " is not implemented yet");

#ifdef MODULE_ACTUATORS
    bool hasActuators = false;
#endif // MODULE_ACTUATORS

    biorbd::utils::String name;
    try {
        while(file.read(main_tag)){  // Attempt read into main_tag, return false if it fails
            // Reinitialize some tags
            name = "";
            property_tag = "";
            subproperty_tag = "";

            // Si c'est un segment
            if (!main_tag.tolower().compare("segment")){
                file.read(name);
                biorbd::utils::String parent_str("root");
                biorbd::utils::String trans = "";
                biorbd::utils::String rot = "";
                bool RTinMatrix(true);
                if (version == 3) // Par défaut pour la version 3 (pas en matrice)
                    RTinMatrix = false;
                bool isRTset(false);
                double mass = 0.00000001;
                Eigen::Matrix3d inertia(Eigen::Matrix3d::Identity(3,3));
                RigidBodyDynamics::Math::Matrix3d RT_R(Eigen::Matrix3d::Identity(3,3));
                RigidBodyDynamics::Math::Vector3d RT_T(0,0,0);
                biorbd::utils::Node3d com(0,0,0);
                biorbd::rigidbody::BoneMesh boneMesh;
                int boneByFile(-1); // -1 non setté, 0 pas par file, 1 par file
                int PF = -1;
                while(file.read(property_tag) && property_tag.tolower().compare("endsegment")){
                    if (!property_tag.tolower().compare("parent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(parent_str);
                        if (parent_str.tolower().compare("root"))
                            biorbd::utils::Error::check(model->GetBodyId(parent_str.c_str()), "Wrong name in a segment");
                    }
                    else if (!property_tag.tolower().compare("translations"))
                        file.read(trans);
                    else if (!property_tag.tolower().compare("rotations"))
                        file.read(rot);
                    else if (!property_tag.tolower().compare("mass"))
                         file.read(mass, variable);
                    else if (!property_tag.tolower().compare("inertia")){
                        Eigen::Matrix3d inertia_tp(Eigen::Matrix3d::Identity(3,3));
                        for (unsigned int i=0; i<9;++i)
                            file.read(inertia_tp(i), variable);
                        inertia = inertia_tp.transpose();
                    }
                    else if (!property_tag.tolower().compare("rtinmatrix")){
                        biorbd::utils::Error::check(isRTset==false, "RT should not appear before RTinMatrix");
                        file.read(RTinMatrix);
                    }
                    else if (!property_tag.tolower().compare("rt")){
                        if (RTinMatrix){ // Matrice 4x4
                            // Compteur pour classification
                            unsigned int cmp_M = 0;
                            unsigned int cmp_T = 0;
                            for (unsigned int i=0; i<12;++i){
                                if ((i+1)%4){
                                    file.read(RT_R(cmp_M), variable);
                                    ++cmp_M;
                                }
                                else{
                                    file.read(RT_T(cmp_T), variable);
                                    ++cmp_T;
                                }
                             }
                        }
                        else{
                            biorbd::utils::String seq("xyz");
                            biorbd::utils::Node3d rot(0, 0, 0);
                            biorbd::utils::Node3d trans(0, 0, 0);
                            // Transcrire les rotations
                            for (unsigned int i=0; i<3; ++i)
                                file.read(rot(i));
                            // Transcrire la séquence d'angle pour les rotations
                            file.read(seq);
                            // Transcrire les translations
                            for (unsigned int i=0; i<3; ++i)
                                file.read(trans(i));
                            biorbd::utils::RotoTrans RT(rot, trans, seq);
                            RT_R = RT.rot().transpose();
                            RT_T = RT.trans();

                        }

                        isRTset = true;
                    }
                    else if (!property_tag.tolower().compare("com"))
                        for (unsigned int i=0; i<3; ++i)
                            file.read(com(i), variable);
                    else if (!property_tag.tolower().compare("forceplate"))
                        file.read(PF);
                    else if (!property_tag.tolower().compare("mesh")){
                        if (boneByFile==-1)
                            boneByFile = 0;
                        else if (boneByFile == 1)
                            biorbd::utils::Error::raise("You must not mix file and mesh in segment");
                        biorbd::utils::Node3d tp(0, 0, 0);
                        for (unsigned int i=0; i<3; ++i)
                            file.read(tp(i), variable);
                        boneMesh.addPoint(tp);
                    }
                    else if (!property_tag.tolower().compare("patch")){
                        if (boneByFile==-1)
                            boneByFile = 0;
                        else if (boneByFile == 1)
                            biorbd::utils::Error::raise("You must not mix file and mesh in segment");
                        biorbd::rigidbody::Patch tp;
                        for (int i=0; i<3; ++i)
                            file.read(tp(i));
                        boneMesh.addPatch(tp);
                    }
                    else if (!property_tag.tolower().compare("meshfile")){
                        if (boneByFile==-1)
                            boneByFile = 1;
                        else if (boneByFile == 0)
                            biorbd::utils::Error::raise("You must not mix file and mesh in segment");
                        biorbd::utils::String filePathInString;
                        file.read(filePathInString);
                        biorbd::utils::Path filePath(filePathInString);
                        if (!filePath.extension().compare("bioMesh"))
                            boneMesh = readBoneMeshFileBiorbdBones(path.folder() + filePath.relativePath());
                        else if (!filePath.extension().compare("ply"))
                            boneMesh = readBoneMeshFilePly(path.folder() + filePath.relativePath());
                        else
                            biorbd::utils::Error::raise(filePath.extension() + " is an unrecognized mesh file");
                    }
                }
                RigidBodyDynamics::Math::SpatialTransform RT(RT_R, RT_T);
                biorbd::rigidbody::BoneCharacteristics characteristics(mass,com,inertia,boneMesh);
                model->AddBone(name, parent_str, trans, rot, characteristics, RT, PF);
            }
            else if (!main_tag.tolower().compare("root_actuated")){
                bool rootActuated = true;
                file.read(rootActuated);
                model->setIsRootActuated(rootActuated);
            }
            else if (!main_tag.tolower().compare("external_forces")){
                bool externalF = false;
                file.read(externalF);
                model->setHasExternalForces(externalF);
            }
            else if (!main_tag.tolower().compare("gravity")){
                biorbd::utils::Node3d gravity(0,0,0);
                for (unsigned int i=0; i<3; ++i)
                    file.read(gravity(i), variable);
                model->gravity = gravity;
            }
            else if (!main_tag.tolower().compare("variables")){
                biorbd::utils::String var;
                while(file.read(var) && var.tolower().compare("endvariables")){
                    if (!var(0).compare("$")){
                        double value;
                        file.read(value);
                        biorbd::utils::Error::check(variable.find(var) == variable.end(), "Variable already defined");
                        variable[var] = value;
                    }
                }
            }
            else if (!main_tag.tolower().compare("marker")){
                biorbd::utils::String name;
                file.read(name);
                unsigned int parent_int = 0;
                biorbd::utils::String parent_str("root");
                biorbd::utils::Node3d pos(0,0,0);
                bool technical = true;
                bool anatomical = false;
                biorbd::utils::String axesToRemove;
                while(file.read(property_tag) && property_tag.tolower().compare("endmarker"))
                    if (!property_tag.tolower().compare("parent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(parent_str);
                        parent_int = model->GetBodyId(parent_str.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(parent_int), "Wrong name in a segment");
                    }
                    else if (!property_tag.tolower().compare("position"))
                        for (unsigned int i=0; i<3; ++i)
                            file.read(pos(i), variable);
                    else if (!property_tag.tolower().compare("technical"))
                        file.read(technical);
                    else if (!property_tag.tolower().compare("anatomical"))
                        file.read(anatomical);
                    else if (!property_tag.tolower().compare("axestoremove"))
                        file.read(axesToRemove);

                model->addMarker(pos, name, parent_str, technical, anatomical, axesToRemove, static_cast<int>(parent_int));
            }
            else if (!main_tag.tolower().compare("mimu") && version >= 4){
                biorbd::utils::Error::raise("MIMU is no more the right tag, change it to IMU!");
            }
            else if (!main_tag.tolower().compare("imu") || !main_tag.tolower().compare("mimu")){
                biorbd::utils::String name;
                file.read(name);
                biorbd::utils::String parent_str("root");
                biorbd::utils::RotoTransNode RT;
                bool RTinMatrix(true);
                if (version == 3) // Par défaut pour la version 3 (pas en matrice)
                    RTinMatrix = false;
                bool isRTset(false);
                bool technical = true;
                bool anatomical = false;
                while(file.read(property_tag) && !(!property_tag.tolower().compare("endimu")
                                                   || !property_tag.tolower().compare("endmimu"))){
                    if (!property_tag.tolower().compare("parent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(parent_str);
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(model->GetBodyId(parent_str.c_str())), "Wrong name in a segment");
                    }
                    else if (!property_tag.tolower().compare("rtinmatrix")){
                        biorbd::utils::Error::check(isRTset==false, "RT should not appear before RTinMatrix");
                        file.read(RTinMatrix);
                    }
                    else if (!property_tag.tolower().compare("rt")){
                        if (RTinMatrix){ // Matrice 4x4
                            for (unsigned int i=0; i<4;++i)
                                for (unsigned int j=0; j<4;++j)
                                        file.read(RT(i,j), variable);
                        }
                        else {
                            biorbd::utils::String seq("xyz");
                            biorbd::utils::Node3d rot(0, 0, 0);
                            biorbd::utils::Node3d trans(0, 0, 0);
                            // Transcrire les rotations
                            for (unsigned int i=0; i<3; ++i)
                                file.read(rot(i));
                            // Transcrire la séquence d'angle pour les rotations
                            file.read(seq);
                            // Transcrire les translations
                            for (unsigned int i=0; i<3; ++i)
                                file.read(trans(i));
                            RT = biorbd::utils::RotoTrans(rot, trans, seq);
                        }
                        isRTset = true;
                    }
                    else if (!property_tag.tolower().compare("technical"))
                        file.read(technical);
                    else if (!property_tag.tolower().compare("anatomical"))
                        file.read(anatomical);
                    RT.setName(name);
                    RT.setParent(parent_str);
                    model->addIMU(RT, technical, anatomical);
                }
            }
            else if (!main_tag.tolower().compare("contact")){
                biorbd::utils::String name;
                file.read(name);
                unsigned int parent_int = 0;
                biorbd::utils::String parent_str("root");
                biorbd::utils::Node3d pos(0,0,0);
                biorbd::utils::Node3d norm(0,0,0);
                biorbd::utils::String axis("");
                double acc = 0;
                while(file.read(property_tag) && property_tag.tolower().compare("endcontact")){
                    if (!property_tag.tolower().compare("parent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(parent_str);
                        parent_int = model->GetBodyId(parent_str.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(parent_int), "Wrong name in a segment");
                    }
                    else if (!property_tag.tolower().compare("position"))
                        for (unsigned int i=0; i<3; ++i)
                            file.read(pos(i), variable);
                    else if (!property_tag.tolower().compare("normal"))
                        for (unsigned int i=0; i<3; ++i)
                            file.read(norm(i), variable);
                    else if (!property_tag.tolower().compare("axis"))
                        file.read(axis);
                    else if (!property_tag.tolower().compare("acceleration"))
                            file.read(acc, variable);
                }
                if (version == 1){
                    biorbd::utils::Error::check(norm.norm() == 1.0, "Normal of the contact must be provided" );
                    model->AddConstraint(parent_int, pos, norm, name, acc);
                }
                else if (version >= 2){
                    biorbd::utils::Error::check(axis.compare(""), "Axis must be provided");
                    model->AddConstraint(parent_int, pos, axis, name, acc);
                }
            }
            else if (!main_tag.tolower().compare("loopconstraint")){
                biorbd::utils::String name;
                unsigned int id_predecessor = 0;
                unsigned int id_successor = 0;
                biorbd::utils::String predecessor_str("root");
                biorbd::utils::String successor_str("root");
                biorbd::utils::RotoTrans X_predecessor;
                biorbd::utils::RotoTrans X_successor;
                biorbd::utils::Vector axis(6);
                bool enableStabilization(false);
                double stabilizationParam(-1);
                while(file.read(property_tag) && property_tag.tolower().compare("endloopconstraint")){
                    if (!property_tag.tolower().compare("predecessor")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(predecessor_str);
                        id_predecessor = model->GetBodyId(predecessor_str.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(id_predecessor), "Wrong name in a segment");
                    }
                    if (!property_tag.tolower().compare("successor")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(successor_str);
                        id_successor = model->GetBodyId(successor_str.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(id_successor), "Wrong name in a segment");
                    } else if (!property_tag.tolower().compare("rtpredecessor")){
                        biorbd::utils::String seq("xyz");
                        biorbd::utils::Node3d rot(0, 0, 0);
                        biorbd::utils::Node3d trans(0, 0, 0);
                        // Transcrire les rotations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(rot(i));
                        // Transcrire la séquence d'angle pour les rotations
                        file.read(seq);
                        // Transcrire les translations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(trans(i));
                        X_predecessor = biorbd::utils::RotoTrans(rot, trans, seq);
                    } else if (!property_tag.tolower().compare("rtsuccessor")){
                        biorbd::utils::String seq("xyz");
                        biorbd::utils::Node3d rot(0, 0, 0);
                        biorbd::utils::Node3d trans(0, 0, 0);
                        // Transcrire les rotations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(rot(i));
                        // Transcrire la séquence d'angle pour les rotations
                        file.read(seq);
                        // Transcrire les translations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(trans(i));
                        X_successor = biorbd::utils::RotoTrans(rot, trans, seq);
                    } else if (!property_tag.tolower().compare("axis"))
                        for (unsigned int i=0; i<axis.size(); ++i)
                            file.read(axis(i), variable);
                    else if (!property_tag.tolower().compare("stabilizationparameter"))
                        file.read(stabilizationParam, variable);
                }
                if (stabilizationParam > 0)
                    enableStabilization = true;
                name = "Loop_" + predecessor_str + "_" + successor_str;
                model->AddLoopConstraint(id_predecessor, id_successor, X_predecessor, X_successor,
                                         axis, name, enableStabilization, stabilizationParam);
            }
            else if (!main_tag.tolower().compare("actuator")) {
    #ifdef MODULE_ACTUATORS
                hasActuators = true;
                // Le nom de l'actuator doit correspondre au numéro du segment sur lequel il s'attache
                biorbd::utils::String name;
                file.read(name);
                unsigned int parent_int = model->GetBodyId(name.c_str());
                // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                biorbd::utils::Error::check(model->IsBodyId(parent_int), "Wrong name in a segment");

                // Declaration de tous les parametres pour tous les types
                biorbd::utils::String type;         bool isTypeSet  = false;
                unsigned int dofIdx(INT_MAX);             bool isDofSet   = false;
                biorbd::utils::String str_direction;bool isDirectionSet = false;
                int int_direction = 0;
                double Tmax(-1);            bool isTmaxSet  = false;
                double T0(-1);              bool isT0Set    = false;
                double pente(-1);           bool isPenteSet = false;
                double wmax(-1);            bool iswmaxSet  = false;
                double wc(-1);              bool iswcSet    = false;
                double amin(-1);            bool isaminSet  = false;
                double wr(-1);              bool iswrSet    = false;
                double w1(-1);              bool isw1Set    = false;
                double r(-1);               bool isrSet     = false;
                double qopt(-1);            bool isqoptSet  = false;
                double facteur6p(-1);       bool isFacteur6pSet = false;
                double r2(-1);              bool isr2Set     = false;
                double qopt2(-1);           bool isqopt2Set  = false;


                while(file.read(property_tag) && property_tag.tolower().compare("endactuator")){
                    if (!property_tag.tolower().compare("type")){
                        file.read(type);
                        isTypeSet = true;
                    }
                    else if (!property_tag.tolower().compare("dof")){
                        biorbd::utils::String dofName;
                        file.read(dofName);
                        dofIdx = model->getDofIndex(name, dofName);
                        isDofSet = true;
                    }
                    else if (!property_tag.tolower().compare("direction")){
                        file.read(str_direction);
                        biorbd::utils::Error::check(!str_direction.tolower().compare("positive") ||
                                            !str_direction.tolower().compare("negative"),
                                            "Direction should be \"positive\" or \"negative\"");
                        if (!str_direction.tolower().compare("positive"))
                            int_direction = 1;
                        else
                            int_direction = -1;
                        isDirectionSet = true;
                    }
                    else if (!property_tag.tolower().compare("tmax")){
                        file.read(Tmax, variable);
                        isTmaxSet = true;
                    }
                    else if (!property_tag.tolower().compare("t0")){
                        file.read(T0, variable);
                        isT0Set = true;
                    }
                    else if (!property_tag.tolower().compare("pente")){
                        file.read(pente, variable);
                        isPenteSet = true;
                    }
                    else if (!property_tag.tolower().compare("wmax")){
                        file.read(wmax, variable);
                        iswmaxSet = true;
                    }
                    else if (!property_tag.tolower().compare("wc")){
                        file.read(wc, variable);
                        iswcSet = true;
                    }
                    else if (!property_tag.tolower().compare("amin")){
                        file.read(amin, variable);
                        isaminSet = true;
                    }
                    else if (!property_tag.tolower().compare("wr")){
                        file.read(wr, variable);
                        iswrSet = true;
                    }
                    else if (!property_tag.tolower().compare("w1")){
                        file.read(w1, variable);
                        isw1Set = true;
                    }
                    else if (!property_tag.tolower().compare("r")){
                        file.read(r, variable);
                        isrSet = true;
                    }
                    else if (!property_tag.tolower().compare("qopt")){
                        file.read(qopt, variable);
                        isqoptSet = true;
                    }
                    else if (!property_tag.tolower().compare("facteur")){
                        file.read(facteur6p, variable);
                        isFacteur6pSet = true;
                    }
                    else if (!property_tag.tolower().compare("r2")){
                        file.read(r2, variable);
                        isr2Set = true;
                    }
                    else if (!property_tag.tolower().compare("qopt2")){
                        file.read(qopt2, variable);
                        isqopt2Set = true;
                    }
                }
                // Vérifier que tout y est
                biorbd::utils::Error::check(isTypeSet!=0, "Actuator type must be defined");
                biorbd::actuator::Actuator* actuator;

                if (!type.tolower().compare("gauss3p")){
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isTmaxSet && isT0Set && iswmaxSet && iswcSet && isaminSet &&
                                        iswrSet && isw1Set && isrSet && isqoptSet,
                                        "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorGauss3p(int_direction,Tmax,T0,wmax,wc,amin,wr,w1,r,qopt,dofIdx,name);
                }
                else if (!type.tolower().compare("constant")){
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isTmaxSet,
                                        "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorConstant(int_direction,Tmax,dofIdx,name);
                }
                else if (!type.tolower().compare("linear")){
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isPenteSet && isT0Set,
                                        "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorLinear(int_direction,T0,pente,dofIdx,name);
                }
                else if (!type.tolower().compare("gauss6p")){
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isTmaxSet && isT0Set && iswmaxSet && iswcSet && isaminSet &&
                                        iswrSet && isw1Set && isrSet && isqoptSet && isFacteur6pSet && isr2Set && isqopt2Set,
                                        "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorGauss6p(int_direction,Tmax,T0,wmax,wc,amin,wr,w1,r,qopt,facteur6p, r2, qopt2, dofIdx,name);
                }
                else {
                    biorbd::utils::Error::raise("Actuator do not correspond to an implemented one");
                    actuator = new biorbd::actuator::ActuatorConstant(int_direction, Tmax, dofIdx, name); // Échec de compilation sinon
                }

                model->addActuator(*actuator);
                delete actuator;
    #else // MODULE_ACTUATORS
            biorbd::utils::Error::raise("Biorbd was build without the module Actuators but the model defines ones");
    #endif // MODULE_ACTUATORS
            } else if (!main_tag.tolower().compare("musclegroup")){
    #ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Nom du groupe musculaire
                // Déclaration des variables
                biorbd::utils::String origin_parent_str("root");
                biorbd::utils::String insert_parent_str("root");
                // Lecture du fichier
                while(file.read(property_tag) && property_tag.tolower().compare("endmusclegroup")){
                    if (!property_tag.tolower().compare("originparent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(origin_parent_str);
                        unsigned int idx = model->GetBodyId(origin_parent_str.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(idx), "Wrong origin parent name for a muscle");
                    }
                    else if (!property_tag.tolower().compare("insertionparent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(insert_parent_str);
                        unsigned int idx = model->GetBodyId(insert_parent_str.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(idx), "Wrong insertion parent name for a muscle");
                    }
                }
                model->addMuscleGroup(name, origin_parent_str, insert_parent_str);
    #else // MODULE_MUSCLES
            biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a muscle group");
    #endif // MODULE_MUSCLES
            }
            else if (!main_tag.tolower().compare("muscle")){
    #ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Nom du muscle
                // Déclaration des variables
                biorbd::muscles::MUSCLE_TYPE type(biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE);
                biorbd::muscles::STATE_TYPE stateType(biorbd::muscles::STATE_TYPE::NO_STATE_TYPE);
                biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType(biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);
                biorbd::utils::String muscleGroup("");
                int idxGroup(-1);
                biorbd::utils::Node3d origin_pos(0,0,0);
                biorbd::utils::Node3d insert_pos(0,0,0);
                double optimalLength(0);
                double maxForce(0);
                double tendonSlackLength(0);
                double pennAngle(0);
                double maxExcitation(0);
                double maxActivation(0);
                double PCSA(1);
                biorbd::muscles::FatigueParameters fatigueParameters;

                // Lecture du fichier
                while(file.read(property_tag) && property_tag.tolower().compare("endmuscle")){
                    if (!property_tag.tolower().compare("musclegroup")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(muscleGroup);
                        idxGroup = model->getGroupId(muscleGroup);
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(idxGroup!=-1, "Could not find muscle group");
                    }
                    else if (!property_tag.tolower().compare("type")){
                        biorbd::utils::String tp_type;
                        file.read(tp_type);
                        if (!tp_type.tolower().compare("idealizedactuator"))
                            type = biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR;
                        else if (!tp_type.tolower().compare("hil"))
                            type = biorbd::muscles::MUSCLE_TYPE::HILL;
                        else if (!tp_type.tolower().compare("hillthelen") || !tp_type.tolower().compare("thelen"))
                            type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN;
                        else if (!tp_type.tolower().compare("hillthelenfatigable") || !tp_type.tolower().compare("thelenfatigable"))
                            type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE;
                        else
                            biorbd::utils::Error::raise(property_tag + " is not a valid muscle type");
                    }
                    else if (!property_tag.tolower().compare("statetype")){
                        biorbd::utils::String tp_state;
                        file.read(tp_state);
                        if (!tp_state.tolower().compare("buchanan"))
                            stateType = biorbd::muscles::STATE_TYPE::BUCHANAN;
                        else
                            biorbd::utils::Error::raise(property_tag + " is not a valid muscle state type");
                    }
                    else if (!property_tag.tolower().compare("originposition"))
                        for (unsigned int i=0; i<3; ++i)
                            file.read(origin_pos(i), variable);
                    else if (!property_tag.tolower().compare("insertionposition"))
                        for (unsigned int i=0; i<3; ++i)
                            file.read(insert_pos(i), variable);
                    else if (!property_tag.tolower().compare("optimallength"))
                        file.read(optimalLength, variable);
                    else if (!property_tag.tolower().compare("tendonslacklength"))
                        file.read(tendonSlackLength, variable);
                    else if (!property_tag.tolower().compare("pennationangle"))
                        file.read(pennAngle, variable);
                    else if (!property_tag.tolower().compare("maximalforce"))
                        file.read(maxForce, variable);
                    else if (!property_tag.tolower().compare("maximalexcitation"))
                        file.read(maxExcitation, variable);
                    else if (!property_tag.tolower().compare("pcsa"))
                        file.read(PCSA, variable);
                    else if (!property_tag.tolower().compare("fatigueparameters")){
                        while(file.read(subproperty_tag) && subproperty_tag.tolower().compare("endfatigueparameters")){
                            if (!subproperty_tag.tolower().compare("type")){
                                biorbd::utils::String tp_fatigue_type;
                                file.read(tp_fatigue_type);
                                if (!tp_fatigue_type.tolower().compare("simple"))
                                    dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE;
                                else if (!tp_fatigue_type.tolower().compare("xia"))
                                    dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA;
                                else
                                    biorbd::utils::Error::raise(tp_fatigue_type + " is not a value fatigue parameter type");
                            } else {
                                double param(0);
                                file.read(param);
                                if (!subproperty_tag.tolower().compare("fatiguerate"))
                                    fatigueParameters.fatigueRate(param);
                                else if (!subproperty_tag.tolower().compare("recoveryrate"))
                                    fatigueParameters.recoveryRate(param);
                                else if (!subproperty_tag.tolower().compare("developfactor"))
                                    fatigueParameters.developFactor(param);
                                else if (!subproperty_tag.tolower().compare("recoveryfactor"))
                                    fatigueParameters.recoveryFactor(param);
                            }
                        }
                    }
                }
                biorbd::utils::Error::check(idxGroup!=-1, "No muscle group was provided!");
                biorbd::muscles::Geometry geo(
                            biorbd::utils::Node3d(origin_pos, name + "_origin", model->muscleGroup(static_cast<unsigned int>(idxGroup)).origin()),
                            biorbd::utils::Node3d(insert_pos, name + "_insertion", model->muscleGroup(static_cast<unsigned int>(idxGroup)).insertion()));
                biorbd::muscles::State stateMax(maxExcitation, maxActivation);
                biorbd::muscles::Characteristics characteristics(optimalLength, maxForce, PCSA, tendonSlackLength, pennAngle, stateMax, fatigueParameters);
                model->muscleGroup(static_cast<unsigned int>(idxGroup)).addMuscle(name,type,geo,characteristics,biorbd::muscles::PathChangers(),stateType,dynamicFatigueType);
    #else // MODULE_MUSCLES
            biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a muscle");
    #endif // MODULE_MUSCLES
            }
            else if (!main_tag.tolower().compare("viapoint")){
    #ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Nom du muscle... Éventuellement ajouter groupe musculaire

                // Déclaration des variables
                biorbd::utils::String parent("");
                biorbd::utils::String muscle("");
                biorbd::utils::String musclegroup("");
                int iMuscleGroup(-1);
                int iMuscle(-1);
                biorbd::muscles::ViaPoint position(0,0,0);

                // Lecture du fichier
                while(file.read(property_tag) && property_tag.tolower().compare("endviapoint")){
                    if (!property_tag.tolower().compare("parent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(parent);
                        unsigned int idx = model->GetBodyId(parent.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(idx), "Wrong origin parent name for a muscle");
                    }
                    else if (!property_tag.tolower().compare("muscle"))
                        file.read(muscle);
                    else if (!property_tag.tolower().compare("musclegroup"))
                        file.read(musclegroup);
                    else if (!property_tag.tolower().compare("position"))
                        for (unsigned int i=0; i<3; ++i)
                            file.read(position(i), variable);
                }
                iMuscleGroup = model->getGroupId(musclegroup);
                biorbd::utils::Error::check(iMuscleGroup!=-1, "No muscle group was provided!");
                iMuscle = model->muscleGroup(static_cast<unsigned int>(iMuscleGroup)).muscleID(muscle);
                biorbd::utils::Error::check(iMuscle!=-1, "No muscle was provided!");
                position.setName(name);
                position.setParent(parent);
                model->muscleGroup(static_cast<unsigned int>(iMuscleGroup))
                        .muscle(static_cast<unsigned int>(iMuscle)).addPathObject(position);
    #else // MODULE_MUSCLES
            biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a viapoint");
    #endif // MODULE_MUSCLES
            }
            else if (!main_tag.tolower().compare("wrap")){
    #ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Nom du wrapping

                // Déclaration des variables
                biorbd::utils::String muscle("");
                biorbd::utils::String musclegroup("");
                int iMuscleGroup(-1);
                int iMuscle(-1);
                biorbd::utils::String parent("");
                biorbd::utils::RotoTrans RT;
                double dia(0);
                double length(0);
                int side(1);

                // Lecture du fichier
                while(file.read(property_tag) && property_tag.tolower().compare("endwrapping")){
                    if (!property_tag.tolower().compare("parent")){
                        // Trouver dynamiquement le numéro du parent
                        file.read(parent);
                        unsigned int idx = model->GetBodyId(parent.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        biorbd::utils::Error::check(model->IsBodyId(idx), "Wrong origin parent name for a muscle");
                    }
                    else if (!property_tag.tolower().compare("rt")){
                        for (unsigned int i=0; i<4;++i)
                            for (unsigned int j=0; j<4; ++j)
                                file.read(RT(i,j), variable);
                    }
                    else if (!property_tag.tolower().compare("muscle"))
                        file.read(muscle);
                    else if (!property_tag.tolower().compare("musclegroup"))
                        file.read(musclegroup);
                    else if (!property_tag.tolower().compare("diameter"))
                        file.read(dia, variable);
                    else if (!property_tag.tolower().compare("length"))
                        file.read(length, variable);
                    else if (!property_tag.tolower().compare("wrappingside"))
                        file.read(side);
                }
                biorbd::utils::Error::check(dia != 0.0, "Diameter was not defined");
                biorbd::utils::Error::check(length != 0.0, "Length was not defined");
                biorbd::utils::Error::check(length < 0.0, "Side was not properly defined");
                biorbd::utils::Error::check(parent != "", "Parent was not defined");
                iMuscleGroup = model->getGroupId(musclegroup);
                biorbd::utils::Error::check(iMuscleGroup!=-1, "No muscle group was provided!");
                iMuscle = model->muscleGroup(static_cast<unsigned int>(iMuscleGroup)).muscleID(muscle);
                biorbd::utils::Error::check(iMuscle!=-1, "No muscle was provided!");
                biorbd::muscles::WrappingCylinder cylinder(RT,dia,length,side,name,parent);
                model->muscleGroup(static_cast<unsigned int>(iMuscleGroup)).muscle(static_cast<unsigned int>(iMuscle)).addPathObject(cylinder);
    #else // MODULE_MUSCLES
            biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a wrapping object");
    #endif // MODULE_MUSCLES
            }
        }
    } catch (std::runtime_error message) {
        biorbd::utils::String error_message("Reading of file \"" + path.filename() + "." + path.extension() + "\" failed with the following error:");
        error_message += "\n" + biorbd::utils::String(message.what()) + "\n";
        if (name.compare(""))
            error_message += "Element: " + main_tag + ", named: " + name + "\n";
        if (property_tag.compare("") && property_tag.find_first_of("end") != 0)
            error_message += "Property: " + property_tag + "\n";
        if (subproperty_tag.compare("") && subproperty_tag.find_first_of("end") != 0)
            error_message += "Subproperty: " + subproperty_tag + "\n";

        biorbd::utils::Error::raise(error_message);
    }
#ifdef MODULE_ACTUATORS
    if (hasActuators)
        model->closeActuator();
#endif // MODULE_ACTUATORS
    // Fermer le fichier
    // std::cout << "Model file successfully loaded" << std::endl;
    file.close();
}
std::vector<std::vector<biorbd::utils::Node3d>>
biorbd::Reader::readMarkerDataFile(
        const biorbd::utils::Path &path){
    // Ouverture du fichier
    // std::cout << "Loading marker file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    // Lecture du fichier
    biorbd::utils::String tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Déterminer le nombre de markers
    file.readSpecificTag("nbmark", tp);
    unsigned int nbMark(static_cast<unsigned int>(atoi(tp.c_str())));

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<std::vector<biorbd::utils::Node3d>> markers;
    std::vector<biorbd::utils::Node3d> position;
    // Descendre jusqu'à la définition d'un markeur
    for (unsigned int j=0; j<nbMark; j++){
        while (tp.compare("Marker")){
            bool check = file.read(tp);
            biorbd::utils::Error::check(check, "Marker file error, wrong size of marker or intervals?");
        }

        unsigned int noMarker;
        file.read(noMarker);
        for (unsigned int i=0; i<=nbIntervals; i++){ // <= parce qu'il y a nbIntervals+1 valeurs
            biorbd::utils::Node3d mark(0, 0, 0);
            for (unsigned int j=0; j<3; ++j)
                file.read(mark[j]);
            position.push_back(mark);
        }
        markers.push_back(position);
        // réinitiation pour la prochaine itération
        tp = "";
        position.clear();
    }

    // Fermer le fichier
    // std::cout << "Marker file successfully loaded" << std::endl;
    file.close();
    return markers;
}

std::vector<biorbd::rigidbody::GeneralizedCoordinates>
biorbd::Reader::readQDataFile(
        const utils::Path &path){
    // Ouverture du fichier
    // std::cout << "Loading kin file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    // Lecture du fichier
    biorbd::utils::String tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Déterminer le nombre de markers
    file.readSpecificTag("nddl", tp);
    unsigned int NDDL(static_cast<unsigned int>(atoi(tp.c_str())));

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::rigidbody::GeneralizedCoordinates> kinematics;
    // Descendre jusqu'à la définition d'un markeur
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            biorbd::utils::Error::check(check, "Kin file error, wrong size of NDDL or intervals?");
        }

        double time;
        file.read(time);
        biorbd::rigidbody::GeneralizedCoordinates position(NDDL);
        for (unsigned int i=0; i<NDDL; i++)
            file.read(position(i));

        kinematics.push_back(position);
        // réinitiation pour la prochaine itération
        tp = "";
    }

    // Fermer le fichier
    // std::cout << "Kin file successfully loaded" << std::endl;
    file.close();
    return kinematics;
}

std::vector<biorbd::utils::Vector>
biorbd::Reader::readActivationDataFile(
        const utils::Path &path){
    // Ouverture du fichier
    // std::cout << "Loading kin file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    // Lecture du fichier
    biorbd::utils::String tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Déterminer le nombre de markers
    file.readSpecificTag("nbmuscles", tp);
    unsigned int nMus(static_cast<unsigned int>(atoi(tp.c_str())));

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::utils::Vector> activations;
    // Descendre jusqu'à la définition d'un markeur
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            biorbd::utils::Error::check(check, "Kin file error, wrong size of number of muscles or intervals?");
        }

        double time;
        file.read(time);
        biorbd::utils::Vector activation_tp(nMus);
        for (unsigned int i=0; i<nMus; i++)
            file.read(activation_tp(i));

        activations.push_back(activation_tp);
        // réinitiation pour la prochaine itération
        tp = "";
    }

    // Fermer le fichier
    // std::cout << "Activation file successfully loaded" << std::endl;
    file.close();
    return activations;
}

std::vector<biorbd::utils::Vector>
biorbd::Reader::readTorqueDataFile(
        const utils::Path &path){
    // Ouverture du fichier
    // std::cout << "Loading kin file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    // Lecture du fichier
    biorbd::utils::String tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Déterminer le nombre de GeneralizedTorque
    file.readSpecificTag("nGeneralizedTorque", tp); //
    unsigned int nGeneralizedTorque(static_cast<unsigned int>(atoi(tp.c_str())));

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::utils::Vector> torque;
    // Descendre jusqu'à la définition d'un torque
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            biorbd::utils::Error::check(check, "Kin file error, wrong size of NGeneralizedTorque or intervals?"); //
        }

        // Lire la première ligne qui est le timestamp
        double time;
        file.read(time);

        // Lire le vecteur de GeneralizedTorque associé au time stamp
        biorbd::utils::Vector torque_tp(nGeneralizedTorque);
        for (unsigned int i=0; i<nGeneralizedTorque; i++)
            file.read(torque_tp(i));

        torque.push_back(torque_tp);
        // réinitiation pour la prochaine itération
        tp = "";
    }

    // Fermer le fichier
    file.close();
    return torque;

}

std::vector<biorbd::utils::Vector>
biorbd::Reader::readGrfDataFile(
        const utils::Path &path){
    // Ouverture du fichier
    // std::cout << "Loading grf file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif
 
    // Lecture du fichier 
    biorbd::utils::String tp;

    // DÃ©terminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // DÃ©terminer le nombre de grf
    file.readSpecificTag("ngrf", tp); //
    unsigned int NGRF(static_cast<unsigned int>(atoi(tp.c_str())));

    // DÃ©terminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::utils::Vector> grf;
    // Descendre jusqu'Ã  la dÃ©finition d'un torque
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            biorbd::utils::Error::check(check, "Grf file error, wrong size of NR or intervals?"); //
        }

        // Lire la premiÃ¨re ligne qui est le timestamp
        double time;
        file.read(time);

        // Lire le vecteur de GeneralizedTorque associÃ© au time stamp
        biorbd::utils::Vector grf_tp(NGRF);
        for (unsigned int i=0; i<NGRF; i++)
            file.read(grf_tp(i));

        grf.push_back(grf_tp);
        // rÃ©initiation pour la prochaine itÃ©ration
        tp = "";
    }

    // Fermer le fichier
    file.close();
    return grf;

}

void biorbd::Reader::readViconForceFile(
        const utils::Path &path, // Path to the file
        std::vector<std::vector<unsigned int>> &frame, // Time vector * number of pf
        std::vector<unsigned int> &frequency, // Acquisition frequency * number of pf
        std::vector<std::vector<biorbd::utils::Node3d>> &force, // Linear forces (x,y,z) * number of pf
        std::vector<std::vector<biorbd::utils::Node3d>> &moment, // Moments (x,y,z) * number of pf
        std::vector<std::vector<biorbd::utils::Node3d>> &cop){// Center of pressure (x,y,z) * number of pf
    // Ouverture du fichier
    // std::cout << "Loading force file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    frame.clear();
    force.clear();
    moment.clear();
    cop.clear();

    // Lecture du fichier
    biorbd::utils::String tp;


    while(!file.eof()){
        // Mettre tout en temporaire par plate-forme
        std::vector<unsigned int> frame1pf;
        std::vector<biorbd::utils::Node3d> force1fp;
        std::vector<biorbd::utils::Node3d> moment1fp;
        std::vector<biorbd::utils::Node3d> cop1fp;

        // Connaitre la fréquence d'acquisition
        file.readSpecificTag("devices", tp);
        unsigned int frequency1pf(static_cast<unsigned int>(atoi(tp.c_str())));

        // Passer l'en-tête
        for (unsigned int i=0; i<4; ++i)
            file.getline(tp);

        // Transcrire les valeurs
        while(!file.eof()){  // return false if it fails
            file.getline(tp); // prendre toute la ligne
            // Si la ligne est vide, c'est qu'on a fini avec les forces pour cette plate-forme
            if (!tp.compare("")){
                break;
            }

            // Sinon, on enregistre la ligne
            // now we'll use a stringstream to separate the fields out of the line (comma separated)
            std::stringstream ss( tp );
            biorbd::utils::String field;
            std::vector<double> data;
            data.clear();
            while (getline( ss, field, ',' )){
                // for each field we wish to convert it to a double
                // (since we require that the CSV contains nothing but floating-point values)
                std::stringstream fs( field );
                double f = 0.0;  // (default value is 0.0)
                fs >> f;

                // add the newly-converted field to the end of the record
                data.push_back( f );
            }
            // S'assurer que la ligne faisait le bon nombre d'éléments (2 temps, 3 cop, 3 forces, 3 moments)
            biorbd::utils::Error::check(data.size()==11, "Wrong number of element in a line in the force file");

            // Remplir les champs
            frame1pf.push_back(static_cast<unsigned int>(data[0]));  // Frame stamp
    //        unsigned int subFrame = static_cast<unsigned int>(data[1]); // Subframes (not interesting)
            biorbd::utils::Node3d cop_tp(0, 0, 0); // Centre de pression
            biorbd::utils::Node3d for_tp(0, 0, 0); // Force
            biorbd::utils::Node3d M_tp(0, 0, 0);   // Moment
            for (unsigned int i=0; i<3; ++i){
                cop_tp[i] = data[i+2]/1000; // de mm à m
                for_tp[i] = data[i+5];
                M_tp[i] = data[i+8]/1000; // de Nmm à Nm
            }
            cop1fp.push_back(cop_tp);
            force1fp.push_back(for_tp);
            moment1fp.push_back(M_tp);
        }
        // Push back des valeurs
        frame.push_back(frame1pf);
        frequency.push_back(frequency1pf);
        force.push_back(force1fp);
        moment.push_back(moment1fp);
        cop.push_back(cop1fp);
    }
}

std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>>
biorbd::Reader::readViconForceFile(
        const biorbd::utils::Path &path) {
    // Lire le fichier
    std::vector<std::vector<unsigned int>> frame;
    std::vector<unsigned int> frequency;// Acquisition frequency
    std::vector<std::vector<biorbd::utils::Node3d>> force; // Linear forces (x,y,z)
    std::vector<std::vector<biorbd::utils::Node3d>> moment; // Moments (x,y,z)
    std::vector<std::vector<biorbd::utils::Node3d>> cop; // Center of pressure (x,y,z)
    readViconForceFile(path, frame, frequency, force, moment, cop);

    // Redispatch des valeurs dans un vecteur de SpatialTransform
    std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>> st; // nb plateforme par temps
    for (unsigned int j=0; j<force.size(); ++j){// nb plateforme
        std::vector<RigidBodyDynamics::Math::SpatialVector> tp;
        for (unsigned int i=0; i<force[j].size(); ++i){ // timestamp
            const biorbd::utils::Node3d& f = force[j][i];  // Linear forces (x,y,z)
            const biorbd::utils::Node3d& m = moment[j][i]; // Moments (x,y,z)
            tp.push_back(RigidBodyDynamics::Math::SpatialVector(m[0], m[1], m[2], f[0], f[1], f[2]));
        }
        st.push_back(tp);
    }
    return st;
}

std::vector<std::vector<biorbd::utils::Node3d>>
biorbd::Reader::readViconMarkerFile(
        const utils::Path &path,
        std::vector<biorbd::utils::String> &markOrder,
        int nNodes) {
    // Lire le fichier
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif
    biorbd::utils::String t;


    // Connaitre la fréquence d'acquisition
    // frequency = atoi(findImportantParameter(file, "trajectories").c_str());

    // Récupérer l'ordre des marqueurs dans le fichier
    for (unsigned int i=0; i<3; ++i) // passer l'entete
        file.read(t);
    size_t idx_tp = 0;
    std::vector<unsigned int> idx_init;
    std::vector<unsigned int> idx_end;
    // Trouver les séparateurs (: et ,)
    while (idx_tp < t.length()) {
        idx_tp = t.find(":", idx_tp+1);
        idx_init.push_back(static_cast<unsigned int>(idx_tp));
        idx_end.push_back(static_cast<unsigned int>(t.find(",", idx_tp+1)));
    }
    // Garder les noms entre les séparateurs
    std::vector<biorbd::utils::String> MarkersInFile;
    for (unsigned int i=0; i<idx_init.size()-1; ++i){
        biorbd::utils::String tp;
        for (unsigned int j=*(idx_init.begin()+i)+1; j<*(idx_end.begin()+i); ++j)
            tp.push_back(t.at(j));
        MarkersInFile.push_back(tp);
    }


    // Comparer avec l'ordre donné
    int *ordre;
    ordre = new int[3*MarkersInFile.size()];
    for (int i=0; i<static_cast<int>(3*MarkersInFile.size()); ++i)
        ordre[i] = -1;
    for (int i=0; i<static_cast<int>(markOrder.size()); ++i){
        unsigned int cmp=0;
        biorbd::utils::String m1 = (*(markOrder.begin()+i));
        while (1){
            biorbd::utils::String m2 = (*(MarkersInFile.begin()+cmp));
            if (!m1.compare(m2)){
                ordre[3*cmp] = 3*i;
                ordre[3*cmp+1] = 3*i+1;
                ordre[3*cmp+2] = 3*i+2;
                break;
            }
            else
                ++cmp;
//            biorbd::utils::Error::check(cmp<MarkersInFile.size(), "Le marqueur demandé n'a pas été trouvé dans le fichier!");
            if (cmp>=MarkersInFile.size())
                break;
        }
    }

    // Se rendre aux données
    for (unsigned int i=0; i<4; ++i) // passer l'entete
        file.read(t);

    // Trouver le nombre de frames total
    unsigned int jumps(1);
    unsigned int nbFrames(0);
    if (nNodes != -1){ // Si c'est tous, les jumps sont de 1
        while(!file.eof()){
            file.read(t); // récupérer une ligne
            nbFrames++;
        }
        file.close();
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif
        // passer l'entete
        for (unsigned int i=0; i<7; ++i)
            file.read(t);
        biorbd::utils::Error::check(nNodes!=0 && nNodes!=1
                && static_cast<unsigned int>(nNodes)<=nbFrames,
                                    "nNode should not be 0, 1 or greater "
                                    "than number of frame");
        jumps = nbFrames/static_cast<unsigned int>(nNodes)+1;
    }


    std::vector<std::vector<biorbd::utils::Node3d>> data;
    // now we'll use a stringstream to separate the fields out of the line (comma separated)
    unsigned int cmpFrames(1);
    while(!file.eof()){
        biorbd::utils::Vector data_tp = biorbd::utils::Vector(static_cast<unsigned int>(3*markOrder.size())).setZero();
        std::stringstream ss( t );
        biorbd::utils::String field;
        unsigned int cmp = 0;
        while (getline( ss, field, ',' )){
            // for each field we wish to convert it to a double
            // (since we require that the CSV contains nothing but floating-point values)
            std::stringstream fs( field );
            double f = 0.0;  // (default value is 0.0)
            fs >> f;
            if (cmp>1 && cmp<3*MarkersInFile.size()+2){ // Retirer les timespans et ne pas dépasser
                int idx = ordre[cmp-2];
                if (idx>=0)
                    data_tp(static_cast<unsigned int>(idx)) = f;
            }
            ++cmp;
        }
        // Une fois les markers en ordre, les séparer
        std::vector<biorbd::utils::Node3d> data_tp2;
        for (unsigned int i=0; i<static_cast<unsigned int>(data_tp.size())/3; ++i){
            biorbd::utils::Node3d node(data_tp.block(3*i, 0, 3, 1)/1000);
            data_tp2.push_back(node);
        }
        // Stocker le vecteur de marker a ce temps t
        data.push_back(data_tp2); // Remettre en metre
        for (unsigned int i=0; i<jumps; ++i){
            if (cmpFrames != nbFrames){
                file.read(t); // récupérer une ligne
                cmpFrames++;
            }
            else{
                file.read(t);
                break;
            }
        }
    }

    // Fermer le fichier
    file.close();

    return data;
}

biorbd::rigidbody::BoneMesh
biorbd::Reader::readBoneMeshFileBiorbdBones(
        const biorbd::utils::Path &path)
{
    // Lire un fichier d'os

    // Ouverture du fichier
    // std::cout << "Loading marker file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    // Lecture du fichier
    biorbd::utils::String tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1 || version == 2, "Version not implemented yet");

    // Savoir le nombre de points
    file.readSpecificTag("npoints", tp);
    unsigned int nPoints(static_cast<unsigned int>(atoi(tp.c_str())));
    file.readSpecificTag("nfaces", tp);
    unsigned int nFaces(static_cast<unsigned int>(atoi(tp.c_str())));

    biorbd::rigidbody::BoneMesh mesh;
    mesh.setPath(path);
    // Récupérer tous les points
    for (unsigned int iPoints=0; iPoints < nPoints; ++iPoints){
        biorbd::utils::Node3d nodeTp(0, 0, 0);
        for (unsigned int i=0; i<3; ++i)
            file.read(nodeTp(i));
        mesh.addPoint(nodeTp);
        if (version == 2)
            for (unsigned int i=0; i<3; ++i){
                double dump; // Ignorer les colonnes 4 5 6
                file.read(dump);
            }
    }

    for (unsigned int iPoints=0; iPoints < nFaces; ++iPoints){
        biorbd::rigidbody::Patch patchTp;
        int nVertices;
        file.read(nVertices);
        if (nVertices != 3)
            biorbd::utils::Error::raise("Patches must be 3 vertices!");
        for (int i=0; i<nVertices; ++i)
            file.read(patchTp(i));
        mesh.addPatch(patchTp);
    }
    return mesh;
}


biorbd::rigidbody::BoneMesh biorbd::Reader::readBoneMeshFilePly(
        const biorbd::utils::Path &path)
{
    // Lire un fichier d'os

    // Ouverture du fichier
    // std::cout << "Loading marker file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif

    // Lecture du fichier
    biorbd::utils::String tp;

    // Savoir le nombre de points
    file.reachSpecificTag("element");
    file.readSpecificTag("vertex", tp);
    unsigned int nVertex(static_cast<unsigned int>(atoi(tp.c_str())));
    int nVertexProperties(file.countTagsInAConsecutiveLines("property"));

    // Trouver le nombre de colonne pour les vertex
    file.reachSpecificTag("element");
    file.readSpecificTag("face", tp);
    unsigned int nFaces(static_cast<unsigned int>(atoi(tp.c_str())));
    int nFacesProperties(file.countTagsInAConsecutiveLines("property"));


    // Trouver le nombre de
    file.reachSpecificTag("end_header");

    biorbd::rigidbody::BoneMesh mesh;
    mesh.setPath(path);
    // Récupérer tous les points
    for (unsigned int iPoints=0; iPoints < nVertex; ++iPoints){
        biorbd::utils::Node3d nodeTp(0, 0, 0);
        for (unsigned int i=0; i<3; ++i)
            file.read(nodeTp(i));
        mesh.addPoint(nodeTp);
        // Ignorer les colonnes post XYZ
        for (int i=0; i<nVertexProperties-3; ++i){
            double dump;
            file.read(dump);
        }
    }

    for (unsigned int iPoints=0; iPoints < nFaces; ++iPoints){
        biorbd::rigidbody::Patch patchTp;
        int nVertices;
        file.read(nVertices);
        if (nVertices != 3)
            biorbd::utils::Error::raise("Patches must be 3 vertices!");
        for (int i=0; i<nVertices; ++i)
            file.read(patchTp(i));
        int dump;
        // Retirer s'il y a des colonnes de trop
        for (int i=0; i<nFacesProperties-1; ++i)
            file.read(dump);
        mesh.addPatch(patchTp);
    }
    return mesh;
}


std::vector<std::vector<biorbd::utils::Node3d>>
biorbd::Reader::readViconMarkerFile(
        const biorbd::utils::Path &path,
        int nNodes){
    // Lire le fichier
#ifdef _WIN32
    biorbd::utils::IfStream file(
                biorbd::utils::Path::toWindowsFormat(
                    path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
                path.absolutePath().c_str(), std::ios::in);
#endif
    biorbd::utils::String t;


    // Connaitre la fréquence d'acquisition
    // frequency = atoi(findImportantParameter(file, "trajectories").c_str());

    // Récupérer l'ordre des marqueurs dans le fichier
    for (unsigned int i=0; i<3; ++i) // passer l'entete
        file.read(t);
    size_t idx_tp = 0;
    std::vector<unsigned int> idx_init;
    std::vector<unsigned int> idx_end;
    // Trouver les séparateurs (: et ,)
    while (idx_tp < t.length()) {
        idx_tp = t.find(":", idx_tp+1);
        idx_init.push_back(static_cast<unsigned int>(idx_tp));
        idx_end.push_back(static_cast<unsigned int>(t.find(",", idx_tp+1)));
    }
    // Garder les noms entre les séparateurs
    std::vector<biorbd::utils::String> MarkersInFile;
    for (unsigned int i=0; i<idx_init.size()-1; ++i){
        biorbd::utils::String tp;
        for (unsigned int j=*(idx_init.begin()+i)+1; j<*(idx_end.begin()+i); ++j)
            tp.push_back(t.at(j));
        MarkersInFile.push_back(tp);
    }

    // fermer le fichier
    file.close();

    return readViconMarkerFile(path, MarkersInFile, nNodes);
}
