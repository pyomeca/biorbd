#define BIORBD_API_EXPORTS
#include "ModelWriter.h"

#include <iostream>
#include <fstream>
#include "BiorbdModel.h"
#include "Utils/Path.h"
#include "RigidBody/IMU.h"
#include "RigidBody/NodeBone.h"
#include "RigidBody/Bone.h"

void biorbd::Writer::writeModel(
        biorbd::Model & m,
        const biorbd::utils::Path& pathToWrite){
    biorbd::utils::String sep("\t"); // separator in the file
    biorbd::utils::String com("//"); // commentaire

    // Gérer le cas où le dossier de destination n'existe pas
    if(!pathToWrite.isFolderExist()) {
        pathToWrite.createFolder();
    }

    // Ouvrir le fichier
    std::ofstream biorbdModelFile;
    biorbdModelFile.open(pathToWrite.c_str());

    // Écrire le fichier
    biorbdModelFile << "version 3" << std::endl;
    biorbdModelFile << std::endl;

    // Informations générale
    biorbdModelFile << std::endl;
    biorbdModelFile << com << " General informations" << std::endl;
    biorbdModelFile << "root_actuated" << sep << m.isRootActuated() << std::endl;
    biorbdModelFile << "external_forces" << sep << m.hasExternalForces() << std::endl;
    biorbdModelFile << std::endl;

    // Informations sur les segments
    std::vector<biorbd::utils::Attitude> localJCS = m.localJCS();
    for (unsigned int i = 0; i<m.nbBone(); ++i){
        biorbdModelFile << com << " Informations about " << m.bone(i).name() << " segment" << std::endl;
        biorbdModelFile << sep << com << " Segment" << std::endl;
        biorbdModelFile << sep << "segment" << sep << m.bone(i).name() << std::endl;
        biorbdModelFile << sep << sep << "parent" << sep << m.bone(i).parentName(m) << std::endl;
        biorbdModelFile << sep << sep << "RTinMatrix" << sep << true << std::endl;
        biorbdModelFile << sep << sep << "RT" << std::endl;
        for (unsigned int j=0; j<4; ++j)
            biorbdModelFile << sep << sep << sep << localJCS[i].block(j,0,1,4) << std::endl;
        if (m.bone(i).nDofTrans() > 0)
            biorbdModelFile << sep << sep << "translations" << sep << m.bone(i).seqT() << std::endl;
        if (m.bone(i).nDofRot() > 0)
            biorbdModelFile << sep << sep << "rotations" << sep << m.bone(i).seqR() << std::endl;
        if (m.bone(i).caract().mesh().path().compare("")) // Si ce n'est pas vide
            biorbdModelFile << sep << sep << "meshfile" << sep << m.bone(i).caract().mesh().path() << std::endl;
        biorbdModelFile << sep << "endsegment" << sep << std::endl;
        biorbdModelFile << std::endl;

        // Écrire les éventuels markers
        std::vector<biorbd::rigidbody::NodeBone> markers (m.marker(m,i));
        if (markers.size() > 0){
            biorbdModelFile << sep << com << " Markers" << std::endl;
            for (size_t j = 0; j< markers.size(); ++j){
                biorbdModelFile << sep << "marker" << sep << markers[j].name() << std::endl;
                biorbdModelFile << sep << sep << "parent" << sep << markers[j].parent() << std::endl;
                biorbdModelFile << sep << sep << "position" << sep << markers[j].position().transpose() << std::endl;
                biorbdModelFile << sep << sep << "technical" << sep << markers[j].isTechnical() << std::endl;
                biorbdModelFile << sep << sep << "anatomical" << sep << markers[j].isAnatomical() << std::endl;
                if (markers[j].nAxesToRemove() != 0)
                    biorbdModelFile << sep << sep << "axestoremove" << sep << markers[j].axesToRemove() << std::endl;
                biorbdModelFile << sep << "endmarker" << sep << std::endl;
            }
        }
        biorbdModelFile << std::endl;

        // Écrire les centrales inertiels
        std::vector<biorbd::rigidbody::IMU> imus(m.IMU(m,i));
        if (imus.size() > 0){
            biorbdModelFile << sep << com << " Inertial Magnetic Unit" << std::endl;
            for (size_t j = 0; j< imus.size(); ++j){
                biorbdModelFile << sep << "imu" << sep << imus[j].name() << std::endl;
                biorbdModelFile << sep << sep << "parent" << sep << imus[j].parent() << std::endl;
                biorbdModelFile << sep << sep << "RTinMatrix" << sep << true << std::endl;
                biorbdModelFile << sep << sep << "RT" << std::endl;
                for (unsigned int k=0; k<4; ++k)
                    biorbdModelFile << sep << sep << sep << imus[j].attitude().block(k,0,1,4) << std::endl;
                biorbdModelFile << sep << sep << "technical" << sep << imus[j].isTechnical() << std::endl;
                biorbdModelFile << sep << sep << "anatomical" << sep << imus[j].isAnatomical() << std::endl;
                biorbdModelFile << sep << "endimu" << sep << std::endl;
            }
        }
        biorbdModelFile << std::endl;
		
        biorbdModelFile << std::endl;
    }
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;



    biorbdModelFile << std::endl;
    // Fermeture du fichier
    biorbdModelFile.close();

}
