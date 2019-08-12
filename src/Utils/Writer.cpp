#define BIORBD_API_EXPORTS
#include "Utils/Writer.h"

#include <iostream>
#include <fstream>
#include "s2mMusculoSkeletalModel.h"
#include "Utils/Path.h"
#include "s2mIMU.h"
#include "s2mNodeBone.h"
#include "s2mBone.h"

void biorbd::utils::Writer::writeModel(
        s2mMusculoSkeletalModel & m,
        const biorbd::utils::Path& pathToWrite){
    biorbd::utils::String sep("\t"); // separator in the file
    biorbd::utils::String com("//"); // commentaire

    // Gérer le cas où le dossier de destination n'existe pas
    if(!pathToWrite.isFolderExist()) {
        pathToWrite.createFolder();
    }

    // Ouvrir le fichier
    std::ofstream s2mModelFile;
    s2mModelFile.open(pathToWrite.c_str());

    // Écrire le fichier
    s2mModelFile << "version 3" << std::endl;
    s2mModelFile << std::endl;

    // Informations générale
    s2mModelFile << std::endl;
    s2mModelFile << com << " General informations" << std::endl;
    s2mModelFile << "root_actuated" << sep << m.isRootActuated() << std::endl;
    s2mModelFile << "external_forces" << sep << m.hasExternalForces() << std::endl;
    s2mModelFile << std::endl;

    // Informations sur les segments
    std::vector<Attitude> localJCS = m.localJCS();
    for (unsigned int i = 0; i<m.nbBone(); ++i){
        s2mModelFile << com << " Informations about " << m.bone(i).name() << " segment" << std::endl;
        s2mModelFile << sep << com << " Segment" << std::endl;
        s2mModelFile << sep << "segment" << sep << m.bone(i).name() << std::endl;
        s2mModelFile << sep << sep << "parent" << sep << m.bone(i).parentName(m) << std::endl;
        s2mModelFile << sep << sep << "RTinMatrix" << sep << true << std::endl;
        s2mModelFile << sep << sep << "RT" << std::endl;
        for (unsigned int j=0; j<4; ++j)
            s2mModelFile << sep << sep << sep << localJCS[i].block(j,0,1,4) << std::endl;
        if (m.bone(i).nDofTrans() > 0)
            s2mModelFile << sep << sep << "translations" << sep << m.bone(i).seqT() << std::endl;
        if (m.bone(i).nDofRot() > 0)
            s2mModelFile << sep << sep << "rotations" << sep << m.bone(i).seqR() << std::endl;
        if (m.bone(i).caract().mesh().path().compare("")) // Si ce n'est pas vide
            s2mModelFile << sep << sep << "meshfile" << sep << m.bone(i).caract().mesh().path() << std::endl;
        s2mModelFile << sep << "endsegment" << sep << std::endl;
        s2mModelFile << std::endl;

        // Écrire les éventuels markers
        std::vector<s2mNodeBone> markers (m.marker(m,i));
        if (markers.size() > 0){
            s2mModelFile << sep << com << " Markers" << std::endl;
            for (size_t j = 0; j< markers.size(); ++j){
                s2mModelFile << sep << "marker" << sep << markers[j].name() << std::endl;
                s2mModelFile << sep << sep << "parent" << sep << markers[j].parent() << std::endl;
                s2mModelFile << sep << sep << "position" << sep << markers[j].position().transpose() << std::endl;
                s2mModelFile << sep << sep << "technical" << sep << markers[j].isTechnical() << std::endl;
                s2mModelFile << sep << sep << "anatomical" << sep << markers[j].isAnatomical() << std::endl;
                if (markers[j].nAxesToRemove() != 0)
                    s2mModelFile << sep << sep << "axestoremove" << sep << markers[j].axesToRemove() << std::endl;
                s2mModelFile << sep << "endmarker" << sep << std::endl;
            }
        }
        s2mModelFile << std::endl;

        // Écrire les centrales inertiels
        std::vector<s2mIMU> imus(m.IMU(m,i));
        if (imus.size() > 0){
            s2mModelFile << sep << com << " Inertial Magnetic Unit" << std::endl;
            for (size_t j = 0; j< imus.size(); ++j){
                s2mModelFile << sep << "imu" << sep << imus[j].name() << std::endl;
                s2mModelFile << sep << sep << "parent" << sep << imus[j].parent() << std::endl;
                s2mModelFile << sep << sep << "RTinMatrix" << sep << true << std::endl;
                s2mModelFile << sep << sep << "RT" << std::endl;
                for (unsigned int k=0; k<4; ++k)
                    s2mModelFile << sep << sep << sep << imus[j].attitude().block(k,0,1,4) << std::endl;
                s2mModelFile << sep << sep << "technical" << sep << imus[j].isTechnical() << std::endl;
                s2mModelFile << sep << sep << "anatomical" << sep << imus[j].isAnatomical() << std::endl;
                s2mModelFile << sep << "endimu" << sep << std::endl;
            }
        }
        s2mModelFile << std::endl;
		
        s2mModelFile << std::endl;
    }
    s2mModelFile << std::endl;
    s2mModelFile << std::endl;
    s2mModelFile << std::endl;
    s2mModelFile << std::endl;
    s2mModelFile << std::endl;
    s2mModelFile << std::endl;



    s2mModelFile << std::endl;
    // Fermeture du fichier
    s2mModelFile.close();

}
