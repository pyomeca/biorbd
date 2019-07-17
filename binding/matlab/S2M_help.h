#ifndef MATLAB_S2M_HELP_H
#define MATLAB_S2M_HELP_H

#include <iostream>

void S2M_help(){

    // Afficher toutes les commandes ainsi que leur définition

    std::cout << "Here are the commands you can call from this interface" << std::endl;
    std::cout << std::endl;
    std::cout << "S2M_rbdl('help')" 				<< std::endl << "\t=> show help menu" 												<< std::endl;
    std::cout << "h = S2M_rbdl('new', pathToModel)" << std::endl << "\t=> load a new model, Return a handler (h)" 							<< std::endl;
    std::cout << "S2M_rbdl('delete', h)" 			<< std::endl << "\t=> delete the loaded 'h' model" 									<< std::endl << std::endl;

    std::cout << "S2M_rbdl('gravity, h, GravityVector)" << std::endl << "\t=> Change gravity field for the given 3d column vector" << std::endl;
    std::cout << "nBody = S2M_rbdl('nBody', h)"			<< std::endl << "\t=> Return the number of segments" << std::endl;
    std::cout << "nameBody = S2M_rbdl('nameBody', h)"			<< std::endl << "\t=> Return the name of segments" << std::endl;
    std::cout << "Mass = S2M_rbdl('totalMass', h)" 		<< std::endl << "\t=> Return the total mass of the model 'h'" 						 << std::endl;
    std::cout << "Mass = S2M_rbdl('segmentmass', h, [idx])" 		<< std::endl << "\t=> Return a vector of mass of each segment of the model 'h' or only idxth segment" 						 << std::endl;
    std::cout << "massMatrix = S2M_rbdl('massMatrix', h, Q)" 		<< std::endl << "\t=> Return the mass matrix for the model 'h' at Q" 									<< std::endl;
    std::cout << "nDof = S2M_rbdl('nDof', h)" 				<< std::endl << "\t=> Return the number of Degrees of Freedom (DoF) for model 'h'" 	<< std::endl;
    std::cout << "nameDof = S2M_rbdl('nameDof', h)" 				<< std::endl << "\t=> Return the name of Degrees of Freedom (DoF) for model 'h'" 	 << std::endl;
    std::cout << "nTau = S2M_rbdl('nTau', h)" 			<< std::endl << "\t=> Return the number of Tau DoF for model 'h'" 			 << std::endl;
    std::cout << "nRoot = S2M_rbdl('nRoot', h)" 			<< std::endl << "\t=> Return the number of DoF on root segment for model 'h'" 		<< std::endl << std::endl;

    std::cout << "CoM = S2M_rbdl('CoM', h, Q)" 				<< std::endl << "\t=> Return the center of mass (CoM) for the model 'h' at Q" 							<< std::endl;
    std::cout << "CoMJac = S2M_rbdl('CoMJacobian', h, Q)" 		<< std::endl << "\t=> Return the CoM jacobian for the model 'h' at Q" 	<< std::endl;
    std::cout << "CoMdot = S2M_rbdl('CoMdot', h, Q, Qdot)" 		<< std::endl << "\t=> Return the CoM velocity for the model 'h' at Q and generalized velocities Qdot" 	<< std::endl;
//    std::cout << "CoMddot = S2M_rbdl('CoMddot', h, Q, Qddot)" 		<< std::endl << "\t=> Return the CoM acceleration for the model 'h' at Q and generalized accelerations Qdot" 	<< std::endl;
    std::cout << "a = S2M_rbdl('CoMangularMomentum', h, Q, Qdot)"	<< std::endl << "\t => Return the angular momentum at CoM for Q and Qdot" << std::endl << std::endl;

    std::cout << "GL = S2M_rbdl('globalJCS', h, Q)" 			<< std::endl << "\t=> Return joint coordinate systems (JCS) in global reference of all segment of model 'h' at generalized coordinates Q" << std::endl;
    std::cout << "CoM = S2M_rbdl('segmentCom', h, Q, [i])" 		<< std::endl << "\t=> Return the segmental CoM for the model 'h' at Q. It returns it for all segments unless optional body index i is provided" 	<< std::endl;
    std::cout << "CoMdot = S2M_rbdl('segmentComDot', h, Q, Qdot, [i])" 		<< std::endl << "\t=> Return the segmental CoM velocity for the model 'h' at Q and Qdot. It returns it for all segments unless optional body index i is provided" 	<< std::endl;
    std::cout << "CoMddot = S2M_rbdl('segmentComDDot', h, Q, Qdot, Qddot, [i])" 		<< std::endl << "\t=> Return the segmental CoM acceleration for the model 'h' at Q, Qdot and Qddot. It returns it for all segments unless optional body index i is provided" 	<< std::endl;
    std::cout << "a = S2M_rbdl('segmentAngularMomentum', h, Q, Qdot, [i])"	<< std::endl << "\t => Return the segmental angular momentum at CoM for Q and Qdot in column matrix (sum of over columns should be equal to comAngularMomentum). If 'i is provided, it returns only for this segment" << std::endl;
    std::cout << "I = S2M_rbdl('segmentsInertia', h, Q)" 			<< std::endl << "\t=> Return the 6x6 inertia matrix for all segment for the model 'h' at Q" 			<< std::endl;
    std::cout << "I = S2M_rbdl('segmentsInertiaLocal', h, [idx])" 			<< std::endl << "\t=> Return the 3x3 inertia matrix for all segment for the model 'h'. If idx is provided, only asked segment is returned" 			<< std::endl;
    std::cout << "w = S2M_rbdl('segmentsVelocities', h, Q, Qdot)" 	<< std::endl << "\t=> Return Plücker's velocity vector for each segment of the model 'h' at Q and Qdot" << std::endl << std::endl;

    std::cout << "nTags = S2M_rbdl('nTags', h)" 			<< std::endl << "\t=> Return the number of markers for the model 'h'" 				<< std::endl;
    std::cout << "nameTags = S2M_rbdl('nameTags', h)" 			<< std::endl << "\t=> Return the names of the markers for the model 'h'" 				<< std::endl ;
    std::cout << "nameTags = S2M_rbdl('nameTechnicalTags', h)" 			<< std::endl << "\t=> Return the names of the technical markers for the model 'h'" 				<< std::endl;
    std::cout << "nameTags = S2M_rbdl('nameAnatomicalTags', h)" 			<< std::endl << "\t=> Return the names of the anatomical markers for the model 'h'" 				<< std::endl ;
    std::cout << "T = S2M_rbdl('LocalTags', h, ['all' (default), 'technical', 'anatomical'])" 			<< std::endl << "\t=> Return the position (X,Y,Z) in lcoal reference of 'all', 'technical' or 'anatomical' markers of the model 'h' at Q" << std::endl;
        std::cout << "T = S2M_rbdl('Tags', h, Q)" 			<< std::endl << "\t=> Return the position (X,Y,Z) in global reference of all markers of the model 'h' at Q" << std::endl;
        std::cout << "T = S2M_rbdl('segmentsTags', h, Q)" 	<< std::endl << "\t=> Return Tags for each segment (in cells) of the model 'h' at Q" << std::endl;
    std::cout << "TJ = S2M_rbdl('TagsJacobian', h, Q)"	<< std::endl << "\t=> Return the jacobian matrix of all markers of the model 'h' at Q" << std::endl;
    std::cout << "C = S2M_rbdl('contacts', h, Q)" << std::endl << "\t=> Return the position of contact points (p) of the model 'h' at Q" << std::endl;
    std::cout << "m = S2M_rbdl('mesh', h, Q)" << std::endl << "\t=> Return position of all points of the model meshing (m) of the model 'h' at Q" << std::endl << std::endl;

    std::cout << "IK = S2M_rbdl('ik', h, Mark, Qinit)"      << std::endl << "\t=> Return inverse kinematics of the model 'h' for measured markers Mark (3xNxTime) with initial guess Q" 	<< std::endl;
    std::cout << "Dyn = S2M_rbdl('inverseDynamics', h, Q, Qdot, Qddot, [F])"      << std::endl << "\t=> Return Tau of the model 'h' at Q, Qdot, Qddot and optional Forces (F) [[MOMENT;FORCE] x NB_forcePlates matrix x time]" 	<< std::endl;
    std::cout << "Dyn = S2M_rbdl('nlEffects', h, Q, Qdot)"      << std::endl << "\t=> Return Tau from NonLinear Effects of the model 'h' at Q, Qdot" 	<< std::endl;
    std::cout << "Qddot = S2M_rbdl('forwardDynamics', h, Q, Qdot, Tau, [useContact])" 				<< std::endl << "\t=> Return Qddot of the model 'h' at Q, Qdot and Tau with optional boolean to process dynamic with contact or not" << std::endl;
    std::cout << "Dyn = S2M_rbdl('torqueActivation', h, Q, Qdot, activations)"      << std::endl << "\t=> Return Tau of the model 'h' at Q, Qdot and given torque activation" 	<< std::endl;
    std::cout << "musclesTau = S2M_rbdl('jointTorqueFromForce', h, Q, Qdot, F)" << std::endl << "\t=> Return muscular Tau of the model 'h' generated by an activation e [nMuscle x time] at Q and QDot" << std::endl;
    std::cout << "musclesTau = S2M_rbdl('jointTorqueFromActivation', h, Q, Qdot, a)" << std::endl << "\t=> Return muscular Tau of the model 'h' generated by an activation a [nMuscle x time] at Q and QDot" << std::endl << std::endl;
    std::cout << "musclesTau = S2M_rbdl('jointTorqueFromExcitation', h, Q, Qdot, a)" << std::endl << "\t=> Return muscular Tau of the model 'h' generated by an excitation a [nMuscle x time] at Q and QDot" << std::endl << std::endl;

    std::cout << "nMus = S2M_rbdl('nmuscles', h)" << std::endl << "\t=> Return total number of muscles of model 'h'" << std::endl;
    std::cout << "[groupsNames, musclesNames, pathNames] = S2M_rbdl('musclesNames', h)" << std::endl << "\t=> Return all groups names, muscle names and via points/wrapping names of model 'h'" << std::endl;
    std::cout << "[m, w] = S2M_rbdl('musclepoints', h, Q)" << std::endl << "\t=> Return all muscles points (m) in a cell and all wrapping objects info (w) of model 'h' at Q" << std::endl;
    std::cout << "l = S2M_rbdl('muscleLength', h, Q)" << std::endl << "\t=> Return all muscles length of model 'h' at Q" << std::endl;
    std::cout << "l = S2M_rbdl('muscleVelocity', h, Q, Qdot)" << std::endl << "\t=> Return all muscles velocities of model 'h' at Q, Qdot" << std::endl;
    std::cout << "adot = S2M_rbdl('muscleActivationDot', h, e, a, [alreadyNormalized=false])" << std::endl << "\t=> Return all muscles activation dot of model 'h' at excitation e and activation a, with default excitation already normalized set to false" << std::endl << std::endl;
    return;

}
#endif // MATLAB_S2M_HELP_H
