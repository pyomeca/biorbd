#define BIORBD_API_EXPORTS
#include "../include/s2mRead.h"

s2mRead::s2mRead()
{

}

void s2mRead::pwd(){
	char c[FILENAME_MAX];
    if (GetCurrentDir(c, sizeof(c)))
        std::cout << c << std::endl;
}
/* Public methods */
s2mMusculoSkeletalModel s2mRead::readModelFile(const s2mPath &path){
    // Ajouter les éléments entrés
    s2mMusculoSkeletalModel model;
    readModelFile(path, &model);
    return model;
}

void s2mRead::readModelFile(const s2mPath &path, s2mMusculoSkeletalModel *model)
{	// Ouverture du fichier
    // std::cout << "Loading model file: " << path << std::endl;
    s2mIfStream file(path.c_str(), std::ios::in);

    // Lecture du fichier
    s2mString tp;

    // Variable utilisée pour remplacer les doubles
    std::map<s2mEquation, double> variable;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version(atoi(tp.c_str()));
    s2mError::s2mAssert((version == 1 || version == 2 || version == 3 || version == 4),  "Version not implemented yet");

    bool hasActuators = false;
    while(file.read(tp)){  // Attempt read into x, return false if it fails
        // Si c'est un segment
        if (!tp.tolower().compare("segment")){
            s2mString name;
            file.read(name);
            unsigned int parent_int = 0;
            s2mString parent_str("root");
            s2mString trans = "";
            s2mString rot = "";
            bool RTinMatrix(true);
            if (version == 3) // Par défaut pour la version 3 (pas en matrice)
                RTinMatrix = false;
            bool isRTset(false);
            double mass = 0.00000001;
            Eigen::Matrix3d inertia(Eigen::Matrix3d::Identity(3,3));
            RigidBodyDynamics::Math::Matrix3d RT_R(Eigen::Matrix3d::Identity(3,3));
            RigidBodyDynamics::Math::Vector3d RT_T(0,0,0);
            Eigen::Vector3d com(0,0,0);
            s2mBoneMesh boneMesh;
            int boneByFile(-1); // -1 non setté, 0 pas par file, 1 par file
            int PF = -1;
            while(file.read(tp) && tp.tolower().compare("endsegment")){
                if (!tp.tolower().compare("parent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(parent_str);
                    if (!parent_str.tolower().compare("root"))
                        parent_int = 0;
                    else{
                        parent_int = model->GetBodyId(parent_str.c_str());
                        // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                        s2mError::s2mAssert(model->IsBodyId(parent_int), "Wrong name in a segment");
                    }
                }
                else if (!tp.tolower().compare("translations"))
                    file.read(trans);
                else if (!tp.tolower().compare("rotations"))
                    file.read(rot);
                else if (!tp.tolower().compare("mass"))
                    file.read(mass, variable);
                else if (!tp.tolower().compare("inertia")){
                    Eigen::Matrix3d inertia_tp(Eigen::Matrix3d::Identity(3,3));
                    for (unsigned int i=0; i<9;++i)
                        file.read(inertia_tp(i), variable);
                    inertia = inertia_tp.transpose();
                }
                else if (!tp.tolower().compare("rtinmatrix")){
                    s2mError::s2mAssert(isRTset==false, "RT should not appear before RTinMatrix");
                    file.read(RTinMatrix);
                }
                else if (!tp.tolower().compare("rt")){
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
                        s2mString seq("xyz");
                        s2mNode rot;
                        s2mNode trans;
                        // Transcrire les rotations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(rot(i));
                        // Transcrire la séquence d'angle pour les rotations
                        file.read(seq);
                        // Transcrire les translations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(trans(i));
                        s2mAttitude RT(rot, trans, seq);
                        RT_R = RT.rot().transpose();
                        RT_T = RT.trans();

                    }

                    isRTset = true;
                }
                else if (!tp.tolower().compare("com"))
                    for (unsigned int i=0; i<3; ++i)
                        file.read(com(i), variable);
                else if (!tp.tolower().compare("forceplate"))
                    file.read(PF);
                else if (!tp.tolower().compare("mesh")){
                    if (boneByFile==-1)
                        boneByFile = 0;
                    else if (boneByFile == 1)
                        s2mError::s2mAssert(0, "You must not mix file and mesh in segment");
                    s2mNode tp;
                    for (unsigned int i=0; i<3; ++i)
                        file.read(tp(i), variable);
                    boneMesh.addPoint(tp);
                }
                else if (!tp.tolower().compare("patch")){
                    if (boneByFile==-1)
                        boneByFile = 0;
                    else if (boneByFile == 1)
                        s2mError::s2mAssert(0, "You must not mix file and mesh in segment");
                    s2mPatch tp;
                    for (unsigned int i=0; i<3; ++i)
                        file.read(tp(i));
                    boneMesh.addPatch(tp);
                }
                else if (!tp.tolower().compare("meshfile")){
                    if (boneByFile==-1)
                        boneByFile = 1;
                    else if (boneByFile == 0)
                        s2mError::s2mAssert(0, "You must not mix file and mesh in segment");
                    s2mPath filePath;
                    file.read(filePath);
                    bool wasRelative = false;
                    s2mPath tpIfWasRelative = "";
                    if (!s2mPath::isFileExist(filePath)){ // regarder si on est en chemin relatif
                        wasRelative = true;
                        tpIfWasRelative = filePath;
                        filePath = path.folder() + filePath; // Si oui, le mettre en absolue (ou du moins relatif à "path", ce qui est suffisant)
                    }
                    filePath.parseFileName();
                    if (!filePath.extension().compare("s2mBone"))
                        boneMesh = readBoneMeshFileS2mBones(filePath);
                    else if (!filePath.extension().compare("ply"))
                        boneMesh = readBoneMeshFilePly(filePath);
                    if (wasRelative)
                        boneMesh.setPath(tpIfWasRelative); // enlever la partie non relative
                }

            }
            RigidBodyDynamics::Math::SpatialTransform RT(RT_R, RT_T);
            s2mBoneCaracteristics caract(mass,com,inertia,boneMesh);
            model->AddBone(parent_int, trans, rot, caract, RT, name, PF);
        }
        else if (!tp.tolower().compare("root_actuated")){
            bool rootActuated = true;
            file.read(rootActuated);
            model->setIsRootActuated(rootActuated);
        }
        else if (!tp.tolower().compare("external_forces")){
            bool externalF = false;
            file.read(externalF);
            model->setHasExternalForces(externalF);
        }
        else if (!tp.tolower().compare("gravity")){
            Eigen::Vector3d gravity(0,0,0);
            for (unsigned int i=0; i<3; ++i)
                file.read(gravity(i), variable);
            model->gravity = gravity;
        }
        else if (!tp.tolower().compare("variables")){
            while(file.read(tp) && tp.tolower().compare("endvariables")){
                if (!tp(0).compare("$")){
                    double value;
                    file.read(value);
                    s2mError::s2mAssert(variable.find(tp) == variable.end(), "Variable already defined");
                    variable[tp] = value;
                }
            }
        }
        else if (!tp.tolower().compare("marker")){
            s2mString name;
            file.read(name);
            unsigned int parent_int = 0;
            s2mString parent_str("root");
            Eigen::Vector3d pos(0,0,0);
            bool technical = true;
            bool anatomical = false;
            s2mString axesToRemove;
            while(file.read(tp) && tp.tolower().compare("endmarker")){
                if (!tp.tolower().compare("parent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(parent_str);
                    parent_int = model->GetBodyId(parent_str.c_str());
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(model->IsBodyId(parent_int), "Wrong name in a segment");
                }
                else if (!tp.tolower().compare("position"))
                    for (unsigned int i=0; i<3; ++i)
                        file.read(pos(i), variable);
                else if (!tp.tolower().compare("technical"))
                    file.read(technical);
                else if (!tp.tolower().compare("anatomical"))
                    file.read(anatomical);
                else if (!tp.tolower().compare("axestoremove"))
                    file.read(axesToRemove);

            }
            model->addMarker(pos, name, parent_str, technical, anatomical, axesToRemove, parent_int);
        }
        else if (!tp.tolower().compare("mimu") && version >= 4){
            s2mError::s2mAssert(false, "MIMU is no more the right tag, change it to IMU!");
        }
        else if (!tp.tolower().compare("imu") || !tp.tolower().compare("mimu")){
            s2mString name;
            file.read(name);
            unsigned int parent_int = 0;
            s2mString parent_str("root");
            s2mAttitude RT;
            bool RTinMatrix(true);
            if (version == 3) // Par défaut pour la version 3 (pas en matrice)
                RTinMatrix = false;
            bool isRTset(false);
            bool technical = true;
            bool anatomical = false;
            while(file.read(tp) && !(!tp.tolower().compare("endimu") || !tp.tolower().compare("endmimu"))){
                if (!tp.tolower().compare("parent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(parent_str);
                    parent_int = model->GetBodyId(parent_str.c_str());
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(model->IsBodyId(parent_int), "Wrong name in a segment");
                }
                else if (!tp.tolower().compare("rtinmatrix")){
                    s2mError::s2mAssert(isRTset==false, "RT should not appear before RTinMatrix");
                    file.read(RTinMatrix);
                }
                else if (!tp.tolower().compare("rt")){
                    if (RTinMatrix) // Matrice 4x4
                        for (unsigned int i=0; i<4;++i)
                            for (unsigned int j=0; j<4;++j)
                                file.read(RT(i,j), variable);
                    else{
                        s2mString seq("xyz");
                        s2mNode rot;
                        s2mNode trans;
                        // Transcrire les rotations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(rot(i));
                        // Transcrire la séquence d'angle pour les rotations
                        file.read(seq);
                        // Transcrire les translations
                        for (unsigned int i=0; i<3; ++i)
                            file.read(trans(i));
                        RT = s2mAttitude(rot, trans, seq);
                    }
                    isRTset = true;
                }
                else if (!tp.tolower().compare("technical"))
                    file.read(technical);
                else if (!tp.tolower().compare("anatomical"))
                    file.read(anatomical);
            }
            model->addIMU(RT, name, parent_str, technical, anatomical, parent_int);
        }
        else if (!tp.tolower().compare("contact")){
            s2mString name;
            file.read(name);
            unsigned int parent_int = 0;
            s2mString parent_str("root");
            Eigen::Vector3d pos(0,0,0);
            Eigen::Vector3d norm(0,0,0);
            s2mString axis("");
            double acc = 0;
            while(file.read(tp) && tp.tolower().compare("endcontact")){
                if (!tp.tolower().compare("parent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(parent_str);
                    parent_int = model->GetBodyId(parent_str.c_str());
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(model->IsBodyId(parent_int), "Wrong name in a segment");
                }
                else if (!tp.tolower().compare("position"))
                    for (unsigned int i=0; i<3; ++i)
                        file.read(pos(i), variable);
                else if (!tp.tolower().compare("normal"))
                    for (unsigned int i=0; i<3; ++i)
                        file.read(norm(i), variable);
                else if (!tp.tolower().compare("axis"))
                    file.read(axis);
                else if (!tp.tolower().compare("acceleration"))
                        file.read(acc, variable);
            }
            if (version == 1){
                s2mError::s2mAssert(norm.norm() == 1, "Normal of the contact must be provided" );
                model->AddConstraint(parent_int, pos, norm, name, acc);
            }
            else if (version >= 2){
                s2mError::s2mAssert(axis.compare(""), "Axis must be provided");
                model->AddConstraint(parent_int, pos, axis, name, acc);
            }
        }
        else if (!tp.tolower().compare("actuator")){
            hasActuators = true;
            // Le nom de l'actuator doit correspondre au numéro du segment sur lequel il s'attache
            s2mString name;
            file.read(name);
            unsigned int parent_int = model->GetBodyId(name.c_str());
            // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
            s2mError::s2mAssert(model->IsBodyId(parent_int), "Wrong name in a segment");

            // Declaration de tous les parametres pour tous les types
            s2mString type;         bool isTypeSet  = false;
            unsigned int dofIdx(INT_MAX);             bool isDofSet   = false;
            s2mString str_direction;bool isDirectionSet = false;
            unsigned int int_direction = 0;
            double Tmax;            bool isTmaxSet  = false;
            double T0;              bool isT0Set    = false;
            double pente;           bool isPenteSet = false;
            double wmax;            bool iswmaxSet  = false;
            double wc;              bool iswcSet    = false;
            double amin;            bool isaminSet  = false;
            double wr;              bool iswrSet    = false;
            double w1;              bool isw1Set    = false;
            double r;               bool isrSet     = false;
            double qopt;            bool isqoptSet  = false;
            double facteur6p;       bool isFacteur6pSet = false;
            double r2;              bool isr2Set     = false;
            double qopt2;           bool isqopt2Set  = false;


            while(file.read(tp) && tp.tolower().compare("endactuator")){
                if (!tp.tolower().compare("type")){
                    file.read(type);
                    isTypeSet = true;
                }
                else if (!tp.tolower().compare("dof")){
                    s2mString dofName;
                    file.read(dofName);
                    dofIdx = model->getDofIndex(name, dofName);
                    isDofSet = true;
                }
                else if (!tp.tolower().compare("direction")){
                    file.read(str_direction);
                    s2mError::s2mAssert(!str_direction.tolower().compare("positive") ||
                                        !str_direction.tolower().compare("negative"),
                                        "Direction should be \"positive\" or \"negative\"");
                    if (!str_direction.tolower().compare("positive"))
                        int_direction = 1;
                    else
                        int_direction = -1;
                    isDirectionSet = true;
                }
                else if (!tp.tolower().compare("tmax")){
                    file.read(Tmax, variable);
                    isTmaxSet = true;
                }
                else if (!tp.tolower().compare("t0")){
                    file.read(T0, variable);
                    isT0Set = true;
                }
                else if (!tp.tolower().compare("pente")){
                    file.read(pente, variable);
                    isPenteSet = true;
                }
                else if (!tp.tolower().compare("wmax")){
                    file.read(wmax, variable);
                    iswmaxSet = true;
                }
                else if (!tp.tolower().compare("wc")){
                    file.read(wc, variable);
                    iswcSet = true;
                }
                else if (!tp.tolower().compare("amin")){
                    file.read(amin, variable);
                    isaminSet = true;
                }
                else if (!tp.tolower().compare("wr")){
                    file.read(wr, variable);
                    iswrSet = true;
                }
                else if (!tp.tolower().compare("w1")){
                    file.read(w1, variable);
                    isw1Set = true;
                }
                else if (!tp.tolower().compare("r")){
                    file.read(r, variable);
                    isrSet = true;
                }
                else if (!tp.tolower().compare("qopt")){
                    file.read(qopt, variable);
                    isqoptSet = true;
                }
                else if (!tp.tolower().compare("facteur")){
                    file.read(facteur6p, variable);
                    isFacteur6pSet = true;
                }
                else if (!tp.tolower().compare("r2")){
                    file.read(r2, variable);
                    isr2Set = true;
                }
                else if (!tp.tolower().compare("qopt2")){
                    file.read(qopt2, variable);
                    isqopt2Set = true;
                }


            }

            // Vérifier que tout y est
            s2mError::s2mAssert(isTypeSet!=0, "Actuator type must be defined");
            s2mActuator* actuator;

            if (!type.tolower().compare("gauss3p")){
                s2mError::s2mAssert(isDofSet && isDirectionSet && isTmaxSet && isT0Set && iswmaxSet && iswcSet && isaminSet &&
                                    iswrSet && isw1Set && isrSet && isqoptSet,
                                    "Make sure all parameters are defined");
                actuator = new s2mActuatorGauss3p(int_direction,Tmax,T0,wmax,wc,amin,wr,w1,r,qopt,dofIdx,name);
            }
            else if (!type.tolower().compare("constant")){
                s2mError::s2mAssert(isDofSet && isDirectionSet && isTmaxSet,
                                    "Make sure all parameters are defined");
                actuator = new s2mActuatorConstant(int_direction,Tmax,dofIdx,name);
            }
            else if (!type.tolower().compare("linear")){
                s2mError::s2mAssert(isDofSet && isDirectionSet && isPenteSet && isT0Set,
                                    "Make sure all parameters are defined");
                actuator = new s2mActuatorLinear(int_direction,T0,pente,dofIdx,name);
            }
            else if (!type.tolower().compare("gauss6p")){
                s2mError::s2mAssert(isDofSet && isDirectionSet && isTmaxSet && isT0Set && iswmaxSet && iswcSet && isaminSet &&
                                    iswrSet && isw1Set && isrSet && isqoptSet && isFacteur6pSet && isr2Set && isqopt2Set,
                                    "Make sure all parameters are defined");
                actuator = new s2mActuatorGauss6p(int_direction,Tmax,T0,wmax,wc,amin,wr,w1,r,qopt,facteur6p, r2, qopt2, dofIdx,name);
            }
            else {
                s2mError::s2mAssert(0, "Actuator do not correspond to an implemented one");
                actuator = new s2mActuatorConstant(int_direction, Tmax, dofIdx, name); // Échec de compilation sinon
            }

            model->addActuator(*model, *actuator);
            delete actuator;

        }
        else if (!tp.tolower().compare("musclegroup")){
            s2mString name;
            file.read(name); // Nom du groupe musculaire

            // Déclaration des variables
            s2mString origin_parent_str("root");
            s2mString insert_parent_str("root");
            // Lecture du fichier
            while(file.read(tp) && tp.tolower().compare("endmusclegroup")){
                if (!tp.tolower().compare("originparent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(origin_parent_str);
                    unsigned int idx = model->GetBodyId(origin_parent_str.c_str());
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(model->IsBodyId(idx), "Wrong origin parent name for a muscle");
                }
                else if (!tp.tolower().compare("insertionparent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(insert_parent_str);
                    unsigned int idx = model->GetBodyId(insert_parent_str.c_str());
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(model->IsBodyId(idx), "Wrong insertion parent name for a muscle");
                }
            }
            model->addMuscleGroup(name, origin_parent_str, insert_parent_str);
        }
        else if (!tp.tolower().compare("muscle")){
            s2mString name;
            file.read(name); // Nom du muscle

            // Déclaration des variables
            s2mString type("");
            s2mString stateType("default");
            s2mString muscleGroup("");
            int idxGroup(-1);
            Eigen::Vector3d origin_pos(0,0,0);
            Eigen::Vector3d insert_pos(0,0,0);
            double optimalLength(0);
            double maxForce(0);
            double tendonSlackLength(0);
            double pennAngle(0);
            double maxExcitation(0);
            double maxActivation(0);
            double PCSA(1);

            // Lecture du fichier
            while(file.read(tp) && tp.tolower().compare("endmuscle")){
                if (!tp.tolower().compare("musclegroup")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(muscleGroup);
                    idxGroup = model->getGroupId(muscleGroup);
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(idxGroup!=-1, "Could not find muscle group");
                }
                else if (!tp.tolower().compare("type"))
                    file.read(type);
                else if (!tp.tolower().compare("statetype"))
                    file.read(stateType);
                else if (!tp.tolower().compare("originposition"))
                    for (unsigned int i=0; i<3; ++i)
                        file.read(origin_pos(i), variable);
                else if (!tp.tolower().compare("insertionposition"))
                    for (unsigned int i=0; i<3; ++i)
                        file.read(insert_pos(i), variable);
                else if (!tp.tolower().compare("optimallength"))
                    file.read(optimalLength, variable);
                else if (!tp.tolower().compare("tendonslacklength"))
                    file.read(tendonSlackLength, variable);
                else if (!tp.tolower().compare("pennationangle"))
                    file.read(pennAngle, variable);
                else if (!tp.tolower().compare("maximalforce"))
                    file.read(maxForce, variable);
                else if (!tp.tolower().compare("maximalexcitation"))
                    file.read(maxExcitation, variable);
                else if (!tp.tolower().compare("pcsa"))
                    file.read(PCSA, variable);
            }
            s2mError::s2mAssert(idxGroup!=-1, "No muscle group was provided!");
            s2mMuscleGeometry geo(s2mNodeMuscle(origin_pos, name + "_origin", model->muscleGroup(idxGroup).origin()),
                                  s2mNodeMuscle(insert_pos, name + "_insertion", model->muscleGroup(idxGroup).insertion()));
            s2mMuscleStateMax state(maxExcitation, maxActivation);
            s2mMuscleCaracteristics caract(optimalLength, maxForce, PCSA, tendonSlackLength, pennAngle, &state);
            model->muscleGroup(idxGroup).addHillMuscle(name,type,geo,caract,s2mMusclePathChangers(),stateType);
        }
        else if (!tp.tolower().compare("viapoint")){
            s2mString name;
            file.read(name); // Nom du muscle... Éventuellement ajouter groupe musculaire

            // Déclaration des variables
            s2mString parent("");
            s2mString muscle("");
            s2mString musclegroup("");
            int iMuscleGroup(-1);
            int iMuscle(-1);
            Eigen::Vector3d position(0,0,0);

            // Lecture du fichier
            while(file.read(tp) && tp.tolower().compare("endviapoint")){
                if (!tp.tolower().compare("parent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(parent);
                    unsigned int idx = model->GetBodyId(parent.c_str());
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(model->IsBodyId(idx), "Wrong origin parent name for a muscle");
                }
                else if (!tp.tolower().compare("muscle"))
                    file.read(muscle);
                else if (!tp.tolower().compare("musclegroup"))
                    file.read(musclegroup);
                else if (!tp.tolower().compare("position"))
                    for (unsigned int i=0; i<3; ++i)
                        file.read(position(i), variable);
            }
            iMuscleGroup = model->getGroupId(musclegroup);
            s2mError::s2mAssert(iMuscleGroup!=-1, "No muscle group was provided!");
            iMuscle = model->muscleGroup(iMuscleGroup).muscleID(muscle);
            s2mError::s2mAssert(iMuscle!=-1, "No muscle was provided!");
            s2mViaPoint v(position,name,parent);
            model->muscleGroup(iMuscleGroup).muscle(iMuscle)->addPathObject(v);
        }
        else if (!tp.tolower().compare("wrap")){
            s2mString name;
            file.read(name); // Nom du wrapping

            // Déclaration des variables
            s2mString muscle("");
            s2mString musclegroup("");
            int iMuscleGroup(-1);
            int iMuscle(-1);
            s2mString parent("");
            Eigen::Matrix4d RT;
            double dia(0);
            double length(0);
            int side(1);

            // Lecture du fichier
            while(file.read(tp) && tp.tolower().compare("endwrapping")){
                if (!tp.tolower().compare("parent")){
                    // Trouver dynamiquement le numéro du parent
                    file.read(parent);
                    unsigned int idx = model->GetBodyId(parent.c_str());
                    // Si parent_int est encore égal à zéro c'est qu'aucun nom a concordé
                    s2mError::s2mAssert(model->IsBodyId(idx), "Wrong origin parent name for a muscle");
                }
                else if (!tp.tolower().compare("rt")){
                    for (unsigned int i=0; i<4;++i)
                        for (unsigned int j=0; j<4; ++j)
                            file.read(RT(i,j), variable);
                }
                else if (!tp.tolower().compare("muscle"))
                    file.read(muscle);
                else if (!tp.tolower().compare("musclegroup"))
                    file.read(musclegroup);
                else if (!tp.tolower().compare("diameter"))
                    file.read(dia, variable);
                else if (!tp.tolower().compare("length"))
                    file.read(length, variable);
                else if (!tp.tolower().compare("wrappingside"))
                    file.read(side);


            }
            s2mError::s2mAssert(dia!=0, "Diameter was not defined");
            s2mError::s2mAssert(length!=0, "Length was not defined");
            s2mError::s2mAssert((length!=-1 || length!=-1), "Side was not properly defined");
            s2mError::s2mAssert(parent != "", "Parent was not defined");
            iMuscleGroup = model->getGroupId(musclegroup);
            s2mError::s2mAssert(iMuscleGroup!=-1, "No muscle group was provided!");
            iMuscle = model->muscleGroup(iMuscleGroup).muscleID(muscle);
            s2mError::s2mAssert(iMuscle!=-1, "No muscle was provided!");
            s2mWrappingCylinder cylinder(RT,dia,length,side,name,parent);
            model->muscleGroup(iMuscleGroup).muscle(iMuscle)->addPathObject(cylinder);
        }
    }

    if (hasActuators)
        model->closeActuator(*model);
    // Fermer le fichier
    // std::cout << "Model file successfully loaded" << std::endl;
    file.close();
}
std::vector<std::vector<Eigen::Vector3d> > s2mRead::readMarkerDataFile(const s2mString &path){
    // Ouverture du fichier
    // std::cout << "Loading marker file: " << path << std::endl;
    s2mIfStream file(path.c_str(), std::ios::in);

    // Lecture du fichier
    s2mString tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version = atoi(tp.c_str());
    s2mError::s2mAssert(version == 1, "Version not implemented yet");

    // Déterminer le nombre de markers
    file.readSpecificTag("nbmark", tp);
    unsigned int nbMark = atoi(tp.c_str());

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals = atoi(tp.c_str());

    std::vector<std::vector<Eigen::Vector3d> > markers;
    std::vector<Eigen::Vector3d> position;
    // Descendre jusqu'à la définition d'un markeur
    for (unsigned int j=0; j<nbMark; j++){
        while (tp.compare("Marker")){
            bool check = file.read(tp);
            s2mError::s2mAssert(check, "Marker file error, wrong size of marker or intervals?");
        }

        unsigned int noMarker;
        file.read(noMarker);
        for (unsigned int i=0; i<=nbIntervals; i++){ // <= parce qu'il y a nbIntervals+1 valeurs
            Eigen::Vector3d mark;
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

std::vector<s2mGenCoord> s2mRead::readQDataFile(const s2mString &path){
    // Ouverture du fichier
    // std::cout << "Loading kin file: " << path << std::endl;
    s2mIfStream file(path.c_str(), std::ios::in);

    // Lecture du fichier
    s2mString tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version = atoi(tp.c_str());
    s2mError::s2mAssert(version == 1, "Version not implemented yet");

    // Déterminer le nombre de markers
    file.readSpecificTag("nddl", tp);
    unsigned int NDDL = atoi(tp.c_str());

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals = atoi(tp.c_str());

    std::vector<s2mGenCoord> kinematics;
    // Descendre jusqu'à la définition d'un markeur
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            s2mError::s2mAssert(check, "Kin file error, wrong size of NDDL or intervals?");
        }

        double time;
        file.read(time);
        s2mGenCoord position(NDDL);
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

std::vector<Eigen::VectorXd> s2mRead::readActivationDataFile(const s2mString &path){
    // Ouverture du fichier
    // std::cout << "Loading kin file: " << path << std::endl;
    s2mIfStream file(path.c_str(), std::ios::in);

    // Lecture du fichier
    s2mString tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version = atoi(tp.c_str());
    s2mError::s2mAssert(version == 1, "Version not implemented yet");

    // Déterminer le nombre de markers
    file.readSpecificTag("nbmuscles", tp);
    unsigned int nMus = atoi(tp.c_str());

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals = atoi(tp.c_str());

    std::vector<Eigen::VectorXd> activations;
    // Descendre jusqu'à la définition d'un markeur
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            s2mError::s2mAssert(check, "Kin file error, wrong size of number of muscles or intervals?");
        }

        double time;
        file.read(time);
        Eigen::VectorXd activation_tp(nMus);
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

std::vector<Eigen::VectorXd> s2mRead::readTorqueDataFile(const s2mString &path){
    // Ouverture du fichier
    // std::cout << "Loading kin file: " << path << std::endl;
    s2mIfStream file(path.c_str(), std::ios::in);

    // Lecture du fichier
    s2mString tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version = atoi(tp.c_str());
    s2mError::s2mAssert(version == 1, "Version not implemented yet");

    // Déterminer le nombre de tau
    file.readSpecificTag("ntau", tp); //
    unsigned int nTau = atoi(tp.c_str());

    // Déterminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals = atoi(tp.c_str());

    std::vector<Eigen::VectorXd> torque;
    // Descendre jusqu'à la définition d'un torque
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            s2mError::s2mAssert(check, "Kin file error, wrong size of NTAU or intervals?"); //
        }

        // Lire la première ligne qui est le timestamp
        double time;
        file.read(time);

        // Lire le vecteur de Tau associé au time stamp
        Eigen::VectorXd torque_tp(nTau);
        for (unsigned int i=0; i<nTau; i++)
            file.read(torque_tp(i));

        torque.push_back(torque_tp);
        // réinitiation pour la prochaine itération
        tp = "";
    }

    // Fermer le fichier
    file.close();
    return torque;

}

std::vector<Eigen::VectorXd> s2mRead::readGrfDataFile(const s2mString &path){
    // Ouverture du fichier
    // std::cout << "Loading grf file: " << path << std::endl;
    s2mIfStream file(path.c_str(), std::ios::in);
 
    // Lecture du fichier 
    s2mString tp;

    // DÃ©terminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version = atoi(tp.c_str());
    s2mError::s2mAssert(version == 1, "Version not implemented yet");

    // DÃ©terminer le nombre de grf
    file.readSpecificTag("ngrf", tp); //
    unsigned int NGRF = atoi(tp.c_str());

    // DÃ©terminer le nombre de noeud
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals = atoi(tp.c_str());

    std::vector<Eigen::VectorXd> grf;
    // Descendre jusqu'Ã  la dÃ©finition d'un torque
    for (unsigned int j=0; j<nbIntervals+1; j++){
        while (tp.compare("T")){
            bool check = file.read(tp);
            s2mError::s2mAssert(check, "Grf file error, wrong size of NR or intervals?"); //
        }

        // Lire la premiÃ¨re ligne qui est le timestamp
        double time;
        file.read(time);

        // Lire le vecteur de Tau associÃ© au time stamp
        Eigen::VectorXd grf_tp(NGRF);
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

void s2mRead::readViconForceFile(const s2mString &path, // Path to the file
                                    std::vector<std::vector<unsigned int>> &frame, // Time vector * number of pf
                                    std::vector<unsigned int> &frequency,// Acquisition frequency * number of pf
                                    std::vector<std::vector<Eigen::Vector3d>> &force, // Linear forces (x,y,z) * number of pf
                                    std::vector<std::vector<Eigen::Vector3d>> &moment, // Moments (x,y,z) * number of pf
                                    std::vector<std::vector<Eigen::Vector3d>> &cop){// Center of pressure (x,y,z) * number of pf
    // Ouverture du fichier
    // std::cout << "Loading force file: " << path << std::endl;
    s2mIfStream file(path.c_str(), std::ios::in);

    frame.clear();
    force.clear();
    moment.clear();
    cop.clear();

    // Lecture du fichier
    s2mString tp;


    while(!file.eof()){
        // Mettre tout en temporaire par plate-forme
        std::vector<unsigned int> frame1pf;
        unsigned int frequency1pf;
        std::vector<Eigen::Vector3d> force1fp;
        std::vector<Eigen::Vector3d> moment1fp;
        std::vector<Eigen::Vector3d> cop1fp;

        // Connaitre la fréquence d'acquisition
        file.readSpecificTag("devices", tp);
        frequency1pf = atoi(tp.c_str());

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
            s2mString field;
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
            s2mError::s2mAssert(data.size()==11, "Wrong number of element in a line in the force file");

            // Remplir les champs
            frame1pf.push_back(static_cast<unsigned int>(data[0]));  // Frame stamp
    //        unsigned int subFrame = static_cast<unsigned int>(data[1]); // Subframes (not interesting)
            Eigen::Vector3d cop_tp; // Centre de pression
            Eigen::Vector3d for_tp; // Force
            Eigen::Vector3d M_tp;   // Moment
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

std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>> s2mRead::readViconForceFile(const s2mString &path){
    // Lire le fichier
    std::vector<std::vector<unsigned int>> frame;
    std::vector<unsigned int> frequency;// Acquisition frequency
    std::vector<std::vector<Eigen::Vector3d>> force; // Linear forces (x,y,z)
    std::vector<std::vector<Eigen::Vector3d>> moment; // Moments (x,y,z)
    std::vector<std::vector<Eigen::Vector3d>> cop; // Center of pressure (x,y,z)
    readViconForceFile(path, frame, frequency, force, moment, cop);

    // Redispatch des valeurs dans un vecteur de SpatialTransform
    // Iterator
    std::vector<std::vector<Eigen::Vector3d> >::iterator force_it = force.begin(); // Linear forces (x,y,z)
    std::vector<std::vector<Eigen::Vector3d> >::iterator moment_it = moment.begin(); // Moments (x,y,z)
    // prepare return STL vector
    std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector> > st; // nb plateforme par temps
    for (unsigned int j=0; j<force.size(); ++j){// nb plateforme
        std::vector<RigidBodyDynamics::Math::SpatialVector> tp;
        for (unsigned int i=0; i<(*(force_it+j)).size(); ++i){ // timestamp
            std::vector<Eigen::Vector3d>::iterator f = (*(force_it+j)).begin()+i; // Linear forces (x,y,z)
            std::vector<Eigen::Vector3d>::iterator m = (*(moment_it+j)).begin()+i; // Moments (x,y,z)
            RigidBodyDynamics::Math::SpatialVector st_tp((*m)[0], (*m)[1], (*m)[2], (*f)[0], (*f)[1], (*f)[2]);
            tp.push_back(st_tp);
        }
        st.push_back(tp);
    }
    return st;
}

std::vector<std::vector<s2mNode> >  s2mRead::readViconMarkerFile(const s2mString &path, std::vector<s2mString> &markOrder, const int& nNodes){
    // Lire le fichier
    s2mIfStream file(path.c_str(), std::ios::in);
    s2mString t;


    // Connaitre la fréquence d'acquisition
    // frequency = atoi(findImportantParameter(file, "trajectories").c_str());

    // Récupérer l'ordre des marqueurs dans le fichier
    for (unsigned int i=0; i<3; ++i) // passer l'entete
        file.read(t);
    unsigned int idx_tp = 0;
    std::vector<unsigned int> idx_init;
    std::vector<unsigned int> idx_end;
    // Trouver les séparateurs (: et ,)
    while (idx_tp < t.length()) {
        idx_tp = t.find(":", idx_tp+1);
        idx_init.push_back(idx_tp);
        idx_end.push_back(t.find(",", idx_tp+1));
    }
    // Garder les noms entre les séparateurs
    std::vector<s2mString> MarkersInFile;
    for (unsigned int i=0; i<idx_init.size()-1; ++i){
        s2mString tp;
        for (unsigned int j=*(idx_init.begin()+i)+1; j<*(idx_end.begin()+i); ++j)
            tp.push_back(t.at(j));
        MarkersInFile.push_back(tp);
    }


    // Comparer avec l'ordre donné
    int *ordre;
    ordre = new int[3*MarkersInFile.size()];
    for (unsigned int i=0; i<3*MarkersInFile.size(); ++i)
        ordre[i] = -1;
    for (unsigned int i=0; i<markOrder.size(); ++i){
        unsigned int cmp=0;
        s2mString m1 = (*(markOrder.begin()+i));
        while (1){
            s2mString m2 = (*(MarkersInFile.begin()+cmp));
            if (!m1.compare(m2)){
                ordre[3*cmp] = 3*i;
                ordre[3*cmp+1] = 3*i+1;
                ordre[3*cmp+2] = 3*i+2;
                break;
            }
            else
                ++cmp;
//            s2mError::s2mAssert(cmp<MarkersInFile.size(), "Le marqueur demandé n'a pas été trouvé dans le fichier!");
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
        file.open(path.c_str(), std::ios::in);
        // passer l'entete
        for (unsigned int i=0; i<7; ++i)
            file.read(t);
        s2mError::s2mAssert(nNodes!=0 && nNodes!=1 && (unsigned int)nNodes<=nbFrames, "nNode should not be 0, 1 or greater than number of frame");
        jumps = nbFrames/nNodes+1;
    }


    std::vector<std::vector<s2mNode> > data;
    // now we'll use a stringstream to separate the fields out of the line (comma separated)
    unsigned int cmpFrames(1);
    while(!file.eof()){
        Eigen::VectorXd data_tp = Eigen::VectorXd::Zero(3*markOrder.size());
        std::stringstream ss( t );
        s2mString field;
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
                    data_tp(idx) = f;
            }
            ++cmp;
        }
        // Une fois les markers en ordre, les séparer
        std::vector<s2mNode> data_tp2;
        for (unsigned int i=0; i<(unsigned int)data_tp.size()/3; ++i){
            s2mNode node(data_tp.block(3*i,0,3,1)/1000);
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

s2mBoneMesh s2mRead::readBoneMeshFileS2mBones(const s2mPath &path)
{
    // Lire un fichier d'os

    // Ouverture du fichier
    // std::cout << "Loading marker file: " << path << std::endl;
    s2mIfStream file(path, std::ios::in);

    // Lecture du fichier
    s2mString tp;

    // Déterminer la version du fichier
    file.readSpecificTag("version", tp);
    unsigned int version = atoi(tp.c_str());
    s2mError::s2mAssert(version == 1 || version == 2, "Version not implemented yet");

    // Savoir le nombre de points
    file.readSpecificTag("npoints", tp);
    unsigned int nPoints(atoi(tp.c_str()));
    file.readSpecificTag("nfaces", tp);
    unsigned int nFaces(atoi(tp.c_str()));

    s2mBoneMesh mesh;
    mesh.setPath(path);
    // Récupérer tous les points
    for (unsigned int iPoints=0; iPoints < nPoints; ++iPoints){
        s2mNode nodeTp;
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
        s2mPatch patchTp;
        int nVertices;
        file.read(nVertices);
        if (nVertices != 3)
            s2mError::s2mAssert(0, "Patches must be 3 vertices!");
        for (int i=0; i<nVertices; ++i)
            file.read(patchTp(i));
        mesh.addPatch(patchTp);
    }
    return mesh;
}


s2mBoneMesh s2mRead::readBoneMeshFilePly(const s2mPath &path)
{
    // Lire un fichier d'os

    // Ouverture du fichier
    // std::cout << "Loading marker file: " << path << std::endl;
    s2mIfStream file(path, std::ios::in);

    // Lecture du fichier
    s2mString tp;

    // Savoir le nombre de points
    file.reachSpecificTag("element");
    file.readSpecificTag("vertex", tp);
    unsigned int nVertex(atoi(tp.c_str()));
    int nVertexProperties(file.countTagsInAConsecutiveLines("property"));

    // Trouver le nombre de colonne pour les vertex
    file.reachSpecificTag("element");
    file.readSpecificTag("face", tp);
    unsigned int nFaces(atoi(tp.c_str()));
    int nFacesProperties(file.countTagsInAConsecutiveLines("property"));


    // Trouver le nombre de
    file.reachSpecificTag("end_header");

    s2mBoneMesh mesh;
    mesh.setPath(path);
    // Récupérer tous les points
    for (unsigned int iPoints=0; iPoints < nVertex; ++iPoints){
        s2mNode nodeTp;
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
        s2mPatch patchTp;
        int nVertices;
        file.read(nVertices);
        if (nVertices != 3)
            s2mError::s2mAssert(0, "Patches must be 3 vertices!");
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


std::vector<std::vector<s2mNode>>  s2mRead::readViconMarkerFile(const s2mString &path, int nNodes){
    // Lire le fichier
    s2mIfStream file(path.c_str(), std::ios::in);
    s2mString t;


    // Connaitre la fréquence d'acquisition
    // frequency = atoi(findImportantParameter(file, "trajectories").c_str());

    // Récupérer l'ordre des marqueurs dans le fichier
    for (unsigned int i=0; i<3; ++i) // passer l'entete
        file.read(t);
    unsigned int idx_tp = 0;
    std::vector<unsigned int> idx_init;
    std::vector<unsigned int> idx_end;
    // Trouver les séparateurs (: et ,)
    while (idx_tp < t.length()) {
        idx_tp = t.find(":", idx_tp+1);
        idx_init.push_back(idx_tp);
        idx_end.push_back(t.find(",", idx_tp+1));
    }
    // Garder les noms entre les séparateurs
    std::vector<s2mString> MarkersInFile;
    for (unsigned int i=0; i<idx_init.size()-1; ++i){
        s2mString tp;
        for (unsigned int j=*(idx_init.begin()+i)+1; j<*(idx_end.begin()+i); ++j)
            tp.push_back(t.at(j));
        MarkersInFile.push_back(tp);
    }

    // fermer le fichier
    file.close();

    return readViconMarkerFile(path, MarkersInFile, nNodes);
}
