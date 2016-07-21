/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2012 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btBulletWorldImporter.h"
#include "../BulletFileLoader/btBulletFile.h"

#include "btBulletDynamicsCommon.h"
#ifndef USE_GIMPACT
#include "BulletCollision/Gimpact/btGImpactShape.h"
#endif


//#define USE_INTERNAL_EDGE_UTILITY
#ifdef USE_INTERNAL_EDGE_UTILITY
#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#endif //USE_INTERNAL_EDGE_UTILITY

btBulletWorldImporter::btBulletWorldImporter(btDynamicsWorld* world)
	:btWorldImporter(world),m_softRigidWorld(dynamic_cast<btSoftRigidDynamicsWorld*>(world))
{

}

btBulletWorldImporter::~btBulletWorldImporter()
{
}


bool	btBulletWorldImporter::loadFile( const char* fileName, const char* preSwapFilenameOut)
{
	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(fileName);


	bool result = loadFileFromMemory(bulletFile2);
	//now you could save the file in 'native' format using
	//bulletFile2->writeFile("native.bullet");
	if (result)
	{
		if (preSwapFilenameOut)
		{
			bulletFile2->preSwap();
			bulletFile2->writeFile(preSwapFilenameOut);
		}

	}
	delete bulletFile2;

	return result;

}



bool	btBulletWorldImporter::loadFileFromMemory( char* memoryBuffer, int len)
{
	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(memoryBuffer,len);

	bool result = loadFileFromMemory(bulletFile2);

	delete bulletFile2;

	return result;
}




bool	btBulletWorldImporter::loadFileFromMemory(  bParse::btBulletFile* bulletFile2)
{
	bool ok = (bulletFile2->getFlags()& bParse::FD_OK)!=0;

	if (ok)
		bulletFile2->parse(m_verboseMode);
	else
		return false;

	if (m_verboseMode & bParse::FD_VERBOSE_DUMP_CHUNKS)
	{
		bulletFile2->dumpChunks(bulletFile2->getFileDNA());
	}

	return convertAllObjects(bulletFile2);

}

bool	btBulletWorldImporter::convertAllObjects(  bParse::btBulletFile* bulletFile2)
{

	m_shapeMap.clear();
	m_bodyMap.clear();

	int i;

	for (i=0;i<bulletFile2->m_bvhs.size();i++)
	{
		btOptimizedBvh* bvh = createOptimizedBvh();

		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btQuantizedBvhDoubleData* bvhData = (btQuantizedBvhDoubleData*)bulletFile2->m_bvhs[i];
			bvh->deSerializeDouble(*bvhData);
		} else
		{
			btQuantizedBvhFloatData* bvhData = (btQuantizedBvhFloatData*)bulletFile2->m_bvhs[i];
			bvh->deSerializeFloat(*bvhData);
		}
		m_bvhMap.insert(bulletFile2->m_bvhs[i],bvh);
	}





	for (i=0;i<bulletFile2->m_collisionShapes.size();i++)
	{
		btCollisionShapeData* shapeData = (btCollisionShapeData*)bulletFile2->m_collisionShapes[i];
		btCollisionShape* shape = convertCollisionShape(shapeData);
		if (shape)
		{
	//		printf("shapeMap.insert(%x,%x)\n",shapeData,shape);
			m_shapeMap.insert(shapeData,shape);
		}

		if (shape&& shapeData->m_name)
		{
			char* newname = duplicateName(shapeData->m_name);
			m_objectNameMap.insert(shape,newname);
			m_nameShapeMap.insert(newname,shape);
		}
	}





	for (int i=0;i<bulletFile2->m_dynamicsWorldInfo.size();i++)
	{
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btDynamicsWorldDoubleData* solverInfoData = (btDynamicsWorldDoubleData*)bulletFile2->m_dynamicsWorldInfo[i];
			btContactSolverInfo solverInfo;

			btVector3 gravity;
			gravity.deSerializeDouble(solverInfoData->m_gravity);

			solverInfo.m_tau = btScalar(solverInfoData->m_solverInfo.m_tau);
			solverInfo.m_damping = btScalar(solverInfoData->m_solverInfo.m_damping);
			solverInfo.m_friction = btScalar(solverInfoData->m_solverInfo.m_friction);
			solverInfo.m_timeStep = btScalar(solverInfoData->m_solverInfo.m_timeStep);

			solverInfo.m_restitution = btScalar(solverInfoData->m_solverInfo.m_restitution);
			solverInfo.m_maxErrorReduction = btScalar(solverInfoData->m_solverInfo.m_maxErrorReduction);
			solverInfo.m_sor = btScalar(solverInfoData->m_solverInfo.m_sor);
			solverInfo.m_erp = btScalar(solverInfoData->m_solverInfo.m_erp);

			solverInfo.m_erp2 = btScalar(solverInfoData->m_solverInfo.m_erp2);
			solverInfo.m_globalCfm = btScalar(solverInfoData->m_solverInfo.m_globalCfm);
			solverInfo.m_splitImpulsePenetrationThreshold = btScalar(solverInfoData->m_solverInfo.m_splitImpulsePenetrationThreshold);
			solverInfo.m_splitImpulseTurnErp = btScalar(solverInfoData->m_solverInfo.m_splitImpulseTurnErp);

			solverInfo.m_linearSlop = btScalar(solverInfoData->m_solverInfo.m_linearSlop);
			solverInfo.m_warmstartingFactor = btScalar(solverInfoData->m_solverInfo.m_warmstartingFactor);
			solverInfo.m_maxGyroscopicForce = btScalar(solverInfoData->m_solverInfo.m_maxGyroscopicForce);
			solverInfo.m_singleAxisRollingFrictionThreshold = btScalar(solverInfoData->m_solverInfo.m_singleAxisRollingFrictionThreshold);

			solverInfo.m_numIterations = solverInfoData->m_solverInfo.m_numIterations;
			solverInfo.m_solverMode = solverInfoData->m_solverInfo.m_solverMode;
			solverInfo.m_restingContactRestitutionThreshold = solverInfoData->m_solverInfo.m_restingContactRestitutionThreshold;
			solverInfo.m_minimumSolverBatchSize = solverInfoData->m_solverInfo.m_minimumSolverBatchSize;

			solverInfo.m_splitImpulse = solverInfoData->m_solverInfo.m_splitImpulse;

			setDynamicsWorldInfo(gravity,solverInfo);
		} else
		{
			btDynamicsWorldFloatData* solverInfoData = (btDynamicsWorldFloatData*)bulletFile2->m_dynamicsWorldInfo[i];
			btContactSolverInfo solverInfo;

			btVector3 gravity;
			gravity.deSerializeFloat(solverInfoData->m_gravity);

			solverInfo.m_tau = solverInfoData->m_solverInfo.m_tau;
			solverInfo.m_damping = solverInfoData->m_solverInfo.m_damping;
			solverInfo.m_friction = solverInfoData->m_solverInfo.m_friction;
			solverInfo.m_timeStep = solverInfoData->m_solverInfo.m_timeStep;

			solverInfo.m_restitution = solverInfoData->m_solverInfo.m_restitution;
			solverInfo.m_maxErrorReduction = solverInfoData->m_solverInfo.m_maxErrorReduction;
			solverInfo.m_sor = solverInfoData->m_solverInfo.m_sor;
			solverInfo.m_erp = solverInfoData->m_solverInfo.m_erp;

			solverInfo.m_erp2 = solverInfoData->m_solverInfo.m_erp2;
			solverInfo.m_globalCfm = solverInfoData->m_solverInfo.m_globalCfm;
			solverInfo.m_splitImpulsePenetrationThreshold = solverInfoData->m_solverInfo.m_splitImpulsePenetrationThreshold;
			solverInfo.m_splitImpulseTurnErp = solverInfoData->m_solverInfo.m_splitImpulseTurnErp;

			solverInfo.m_linearSlop = solverInfoData->m_solverInfo.m_linearSlop;
			solverInfo.m_warmstartingFactor = solverInfoData->m_solverInfo.m_warmstartingFactor;
			solverInfo.m_maxGyroscopicForce = solverInfoData->m_solverInfo.m_maxGyroscopicForce;
			solverInfo.m_singleAxisRollingFrictionThreshold = solverInfoData->m_solverInfo.m_singleAxisRollingFrictionThreshold;

			solverInfo.m_numIterations = solverInfoData->m_solverInfo.m_numIterations;
			solverInfo.m_solverMode = solverInfoData->m_solverInfo.m_solverMode;
			solverInfo.m_restingContactRestitutionThreshold = solverInfoData->m_solverInfo.m_restingContactRestitutionThreshold;
			solverInfo.m_minimumSolverBatchSize = solverInfoData->m_solverInfo.m_minimumSolverBatchSize;

			solverInfo.m_splitImpulse = solverInfoData->m_solverInfo.m_splitImpulse;

			setDynamicsWorldInfo(gravity,solverInfo);
		}
	}


	for (i=0;i<bulletFile2->m_rigidBodies.size();i++)
	{
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btRigidBodyDoubleData* colObjData = (btRigidBodyDoubleData*)bulletFile2->m_rigidBodies[i];
			convertRigidBodyDouble(colObjData);
		} else
		{
			btRigidBodyFloatData* colObjData = (btRigidBodyFloatData*)bulletFile2->m_rigidBodies[i];
			convertRigidBodyFloat(colObjData);
		}


	}

	for (i=0;i<bulletFile2->m_collisionObjects.size();i++)
	{
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btCollisionObjectDoubleData* colObjData = (btCollisionObjectDoubleData*)bulletFile2->m_collisionObjects[i];
			btCollisionShape** shapePtr = m_shapeMap.find(colObjData->m_collisionShape);
			if (shapePtr && *shapePtr)
			{
				btTransform startTransform;
				colObjData->m_worldTransform.m_origin.m_floats[3] = 0.f;
				startTransform.deSerializeDouble(colObjData->m_worldTransform);

				btCollisionShape* shape = (btCollisionShape*)*shapePtr;
				btCollisionObject* body = createCollisionObject(startTransform,shape,colObjData->m_name);
				body->setFriction(btScalar(colObjData->m_friction));
				body->setRestitution(btScalar(colObjData->m_restitution));

#ifdef USE_INTERNAL_EDGE_UTILITY
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
					if (trimesh->getTriangleInfoMap())
					{
						body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					}
				}
#endif //USE_INTERNAL_EDGE_UTILITY
				m_bodyMap.insert(colObjData,body);
			} else
			{
				printf("error: no shape found\n");
			}

		} else
		{
			btCollisionObjectFloatData* colObjData = (btCollisionObjectFloatData*)bulletFile2->m_collisionObjects[i];
			btCollisionShape** shapePtr = m_shapeMap.find(colObjData->m_collisionShape);
			if (shapePtr && *shapePtr)
			{
				btTransform startTransform;
				colObjData->m_worldTransform.m_origin.m_floats[3] = 0.f;
				startTransform.deSerializeFloat(colObjData->m_worldTransform);

				btCollisionShape* shape = (btCollisionShape*)*shapePtr;
				btCollisionObject* body = createCollisionObject(startTransform,shape,colObjData->m_name);

#ifdef USE_INTERNAL_EDGE_UTILITY
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
					if (trimesh->getTriangleInfoMap())
					{
						body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					}
				}
#endif //USE_INTERNAL_EDGE_UTILITY
				m_bodyMap.insert(colObjData,body);
			} else
			{
				printf("error: no shape found\n");
			}
		}

	}


	for (i=0;i<bulletFile2->m_constraints.size();i++)
	{
		btTypedConstraintData2* constraintData = (btTypedConstraintData2*)bulletFile2->m_constraints[i];
		btTypedConstraintFloatData* singleC = (btTypedConstraintFloatData*)bulletFile2->m_constraints[i];
		btTypedConstraintDoubleData* doubleC = (btTypedConstraintDoubleData*)bulletFile2->m_constraints[i];

		btCollisionObject** colAptr = m_bodyMap.find(constraintData->m_rbA);
		btCollisionObject** colBptr = m_bodyMap.find(constraintData->m_rbB);

		btRigidBody* rbA = 0;
		btRigidBody* rbB = 0;

		if (colAptr)
		{
			rbA = btRigidBody::upcast(*colAptr);
			if (!rbA)
				rbA = &getFixedBody();
		}
		if (colBptr)
		{
			rbB = btRigidBody::upcast(*colBptr);
			if (!rbB)
				rbB = &getFixedBody();
		}
		if (!rbA && !rbB)
			continue;

		bool isDoublePrecisionData = (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)!=0;

		if (isDoublePrecisionData)
		{
			if (bulletFile2->getVersion()>=282)
			{
				btTypedConstraintDoubleData* dc = (btTypedConstraintDoubleData*)constraintData;
				convertConstraintDouble(dc, rbA,rbB, bulletFile2->getVersion());
			} else
			{
				//double-precision constraints were messed up until 2.82, try to recover data...

				btTypedConstraintData* oldData = (btTypedConstraintData*)constraintData;

				convertConstraintBackwardsCompatible281(oldData, rbA,rbB, bulletFile2->getVersion());

			}
		}
		else
		{
			btTypedConstraintFloatData* dc = (btTypedConstraintFloatData*)constraintData;
			convertConstraintFloat(dc, rbA,rbB, bulletFile2->getVersion());
		}


	}

	//int i;
		///ADDED now the soft bodies
		if(!m_softRigidWorld&&bulletFile2->m_softBodies.size()>0)printf("Can't add softbodies to btRigidWorld\n");
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				int i;
				int numNodes = softBodyData->m_numNodes;


				btSoftBody*		psb=new btSoftBody(&m_softRigidWorld->getWorldInfo());
				m_softBodyMap.insert(softBodyData,psb);

				//materials
				for (i=0;i<softBodyData->m_numMaterials;i++)
				{
					SoftBodyMaterialData* matData = softBodyData->m_materials[i];
					btSoftBody::Material** matPtr = m_materialMap.find(matData);
					btSoftBody::Material* mat = 0;
					if (matPtr&& *matPtr)
					{
						mat = *matPtr;
					} else
					{
						mat = psb->appendMaterial();
						mat->m_flags = matData->m_flags;
						mat->m_kAST = matData->m_angularStiffness;
						mat->m_kLST = matData->m_linearStiffness;
						mat->m_kVST = matData->m_volumeStiffness;
						m_materialMap.insert(matData,mat);
					}
				}




				for (i=0;i<numNodes;i++)
				{
					SoftBodyNodeData& nodeData = softBodyData->m_nodes[i];
					btVector3 position;
					position.deSerializeFloat(nodeData.m_position);
					btScalar mass = nodeData.m_inverseMass? 1./nodeData.m_inverseMass : 0.f;
					psb->appendNode(position,mass);
					btSoftBody::Node* node = &psb->m_nodes[psb->m_nodes.size()-1];
					node->m_area = nodeData.m_area;
					node->m_battach = nodeData.m_attach;
					node->m_f.deSerializeFloat(nodeData.m_accumulatedForce);
					node->m_im = nodeData.m_inverseMass;

					btSoftBody::Material** matPtr = m_materialMap.find(nodeData.m_material);
					if (matPtr && *matPtr)
					{
						node->m_material = *matPtr;
					} else
					{
						printf("no mat?\n");
					}

					node->m_n.deSerializeFloat(nodeData.m_normal);
					node->m_q = node->m_x;
					node->m_v.deSerializeFloat(nodeData.m_velocity);

				}

				for (i=0;i<softBodyData->m_numLinks;i++)
				{
					SoftBodyLinkData& linkData = softBodyData->m_links[i];
					btSoftBody::Material** matPtr = m_materialMap.find(linkData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1],*matPtr);
					} else
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1]);
					}
					btSoftBody::Link* link = &psb->m_links[psb->m_links.size()-1];
					link->m_bbending = linkData.m_bbending;
					link->m_rl = linkData.m_restLength;
				}

				for (i=0;i<softBodyData->m_numFaces;i++)
				{
					SoftBodyFaceData& faceData = softBodyData->m_faces[i];
					btSoftBody::Material** matPtr = m_materialMap.find(faceData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2],*matPtr);
					} else
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2]);
					}
					btSoftBody::Face* face = &psb->m_faces[psb->m_faces.size()-1];
					face->m_normal.deSerializeFloat(faceData.m_normal);
					face->m_ra = faceData.m_restArea;
				}



				//anchors
				for (i=0;i<softBodyData->m_numAnchors;i++)
				{
					btCollisionObject** colAptr = m_bodyMap.find(softBodyData->m_anchors[i].m_rigidBody);
					if (colAptr && *colAptr)
					{
						btRigidBody* body = btRigidBody::upcast(*colAptr);
						if (body)
						{
							bool disableCollision = false;
							btVector3 localPivot;
							localPivot.deSerializeFloat(softBodyData->m_anchors[i].m_localFrame);
							psb->appendAnchor(softBodyData->m_anchors[i].m_nodeIndex,body,localPivot, disableCollision);
						}
					}
				}

				if (softBodyData->m_pose)
				{
					psb->m_pose.m_aqq.deSerializeFloat(  softBodyData->m_pose->m_aqq);
					psb->m_pose.m_bframe = (softBodyData->m_pose->m_bframe!=0);
					psb->m_pose.m_bvolume = (softBodyData->m_pose->m_bvolume!=0);
					psb->m_pose.m_com.deSerializeFloat(softBodyData->m_pose->m_com);

					psb->m_pose.m_pos.resize(softBodyData->m_pose->m_numPositions);
					for (i=0;i<softBodyData->m_pose->m_numPositions;i++)
					{
						psb->m_pose.m_pos[i].deSerializeFloat(softBodyData->m_pose->m_positions[i]);
					}
					psb->m_pose.m_rot.deSerializeFloat(softBodyData->m_pose->m_rot);
					psb->m_pose.m_scl.deSerializeFloat(softBodyData->m_pose->m_scale);
					psb->m_pose.m_wgh.resize(softBodyData->m_pose->m_numWeigts);
					for (i=0;i<softBodyData->m_pose->m_numWeigts;i++)
					{
						psb->m_pose.m_wgh[i] = softBodyData->m_pose->m_weights[i];
					}
					psb->m_pose.m_volume = softBodyData->m_pose->m_restVolume;
				}

#if 1
				psb->m_cfg.piterations=softBodyData->m_config.m_positionIterations;
				psb->m_cfg.diterations=softBodyData->m_config.m_driftIterations;
				psb->m_cfg.citerations=softBodyData->m_config.m_clusterIterations;
				psb->m_cfg.viterations=softBodyData->m_config.m_velocityIterations;

				//psb->setTotalMass(0.1);
				psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_)softBodyData->m_config.m_aeroModel;
				psb->m_cfg.kLF = softBodyData->m_config.m_lift;
				psb->m_cfg.kDG = softBodyData->m_config.m_drag;
				psb->m_cfg.kMT = softBodyData->m_config.m_poseMatch;
				psb->m_cfg.collisions = softBodyData->m_config.m_collisionFlags;
				psb->m_cfg.kDF = 1.f;//softBodyData->m_config.m_dynamicFriction;
				psb->m_cfg.kDP = softBodyData->m_config.m_damping;
				psb->m_cfg.kPR = softBodyData->m_config.m_pressure;
				psb->m_cfg.kVC = softBodyData->m_config.m_volume;
				psb->m_cfg.kAHR = softBodyData->m_config.m_anchorHardness;
				psb->m_cfg.kKHR = softBodyData->m_config.m_kineticContactHardness;
				psb->m_cfg.kSHR = softBodyData->m_config.m_softContactHardness;
				psb->m_cfg.kSRHR_CL = softBodyData->m_config.m_softRigidClusterHardness;
				psb->m_cfg.kSKHR_CL = softBodyData->m_config.m_softKineticClusterHardness;
				psb->m_cfg.kSSHR_CL = softBodyData->m_config.m_softSoftClusterHardness;
#endif

//				pm->m_kLST				=	1;

#if 1
				//clusters
				if (softBodyData->m_numClusters)
				{
					m_clusterBodyMap.insert(softBodyData->m_clusters,psb);
					int j;
					psb->m_clusters.resize(softBodyData->m_numClusters);
					for (i=0;i<softBodyData->m_numClusters;i++)
					{
						psb->m_clusters[i] = new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();
						psb->m_clusters[i]->m_adamping = softBodyData->m_clusters[i].m_adamping;
						psb->m_clusters[i]->m_av.deSerializeFloat(softBodyData->m_clusters[i].m_av);
						psb->m_clusters[i]->m_clusterIndex = softBodyData->m_clusters[i].m_clusterIndex;
						psb->m_clusters[i]->m_collide = (softBodyData->m_clusters[i].m_collide!=0);
						psb->m_clusters[i]->m_com.deSerializeFloat(softBodyData->m_clusters[i].m_com);
						psb->m_clusters[i]->m_containsAnchor = (softBodyData->m_clusters[i].m_containsAnchor!=0);
						psb->m_clusters[i]->m_dimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[0]);
						psb->m_clusters[i]->m_dimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[1]);

						psb->m_clusters[i]->m_framerefs.resize(softBodyData->m_clusters[i].m_numFrameRefs);
						for (j=0;j<softBodyData->m_clusters[i].m_numFrameRefs;j++)
						{
							psb->m_clusters[i]->m_framerefs[j].deSerializeFloat(softBodyData->m_clusters[i].m_framerefs[j]);
						}
						psb->m_clusters[i]->m_nodes.resize(softBodyData->m_clusters[i].m_numNodes);
						for (j=0;j<softBodyData->m_clusters[i].m_numNodes;j++)
						{
							int nodeIndex = softBodyData->m_clusters[i].m_nodeIndices[j];
							psb->m_clusters[i]->m_nodes[j] = &psb->m_nodes[nodeIndex];
						}

						psb->m_clusters[i]->m_masses.resize(softBodyData->m_clusters[i].m_numMasses);
						for (j=0;j<softBodyData->m_clusters[i].m_numMasses;j++)
						{
							psb->m_clusters[i]->m_masses[j] = softBodyData->m_clusters[i].m_masses[j];
						}
						psb->m_clusters[i]->m_framexform.deSerializeFloat(softBodyData->m_clusters[i].m_framexform);
						psb->m_clusters[i]->m_idmass = softBodyData->m_clusters[i].m_idmass;
						psb->m_clusters[i]->m_imass = softBodyData->m_clusters[i].m_imass;
						psb->m_clusters[i]->m_invwi.deSerializeFloat(softBodyData->m_clusters[i].m_invwi);
						psb->m_clusters[i]->m_ldamping = softBodyData->m_clusters[i].m_ldamping;
						psb->m_clusters[i]->m_locii.deSerializeFloat(softBodyData->m_clusters[i].m_locii);
						psb->m_clusters[i]->m_lv.deSerializeFloat(softBodyData->m_clusters[i].m_lv);
						psb->m_clusters[i]->m_matching = softBodyData->m_clusters[i].m_matching;
						psb->m_clusters[i]->m_maxSelfCollisionImpulse = 0;//softBodyData->m_clusters[i].m_maxSelfCollisionImpulse;
						psb->m_clusters[i]->m_ndamping = softBodyData->m_clusters[i].m_ndamping;
						psb->m_clusters[i]->m_ndimpulses = softBodyData->m_clusters[i].m_ndimpulses;
						psb->m_clusters[i]->m_nvimpulses = softBodyData->m_clusters[i].m_nvimpulses;
						psb->m_clusters[i]->m_selfCollisionImpulseFactor = softBodyData->m_clusters[i].m_selfCollisionImpulseFactor;
						psb->m_clusters[i]->m_vimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[0]);
						psb->m_clusters[i]->m_vimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[1]);

					}
					//psb->initializeClusters();
					//psb->updateClusters();

				}
#else

				psb->m_cfg.piterations	=	2;
				psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS;
				//psb->setTotalMass(50,true);
				//psb->generateClusters(64);
				//psb->m_cfg.kDF=1;
				psb->generateClusters(8);


#endif //



				psb->updateConstants();
				m_softRigidWorld->getWorldInfo().m_dispatcher = m_softRigidWorld->getDispatcher();

				m_softRigidWorld->addSoftBody(psb);


			}
		}


		//now the soft body joints
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				btSoftBody** sbp = m_softBodyMap.find(softBodyData);
				if (sbp && *sbp)
				{
					btSoftBody* sb = *sbp;
					for (int i=0;i<softBodyData->m_numJoints;i++)
					{
						btSoftBodyJointData* sbjoint = &softBodyData->m_joints[i];


						btSoftBody::Body bdyB;

						btSoftBody* sbB = 0;
						btTransform transA;
						transA.setIdentity();
						transA = sb->m_clusters[0]->m_framexform;

						btCollisionObject** colBptr = m_bodyMap.find(sbjoint->m_bodyB);
						if (colBptr && *colBptr)
						{
							btRigidBody* rbB = btRigidBody::upcast(*colBptr);
							if (rbB)
							{
								bdyB = rbB;
							} else
							{
								bdyB = *colBptr;
							}
						}


						btSoftBody** bodyBptr = m_clusterBodyMap.find(sbjoint->m_bodyB);
						if (bodyBptr && *bodyBptr )
						{
							sbB = *bodyBptr;
							bdyB = sbB->m_clusters[0];
						}


						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Linear)
						{
							btSoftBody::LJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.position = transA*relA;
							sb->appendLinearJoint(specs,sb->m_clusters[0],bdyB);
						}

						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Angular)
						{
							btSoftBody::AJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.axis = transA.getBasis()*relA;
							sb->appendAngularJoint(specs,sb->m_clusters[0],bdyB);
						}
					}
				}

			}
		}

	return true;
}


 bool MySoftBulletWorldImporter::convertAllObjects(  bParse::btBulletFile* bulletFile2)
	{
		bool result = btBulletWorldImporter::convertAllObjects(bulletFile2);
		int i;
		//now the soft bodies
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				int i;
				int numNodes = softBodyData->m_numNodes;


				btSoftBody*		psb=new btSoftBody(&m_softRigidWorld->getWorldInfo());
				m_softBodyMap.insert(softBodyData,psb);

				//materials
				for (i=0;i<softBodyData->m_numMaterials;i++)
				{
					SoftBodyMaterialData* matData = softBodyData->m_materials[i];
					btSoftBody::Material** matPtr = m_materialMap.find(matData);
					btSoftBody::Material* mat = 0;
					if (matPtr&& *matPtr)
					{
						mat = *matPtr;
					} else
					{
						mat = psb->appendMaterial();
						mat->m_flags = matData->m_flags;
						mat->m_kAST = matData->m_angularStiffness;
						mat->m_kLST = matData->m_linearStiffness;
						mat->m_kVST = matData->m_volumeStiffness;
						m_materialMap.insert(matData,mat);
					}
				}




				for (i=0;i<numNodes;i++)
				{
					SoftBodyNodeData& nodeData = softBodyData->m_nodes[i];
					btVector3 position;
					position.deSerializeFloat(nodeData.m_position);
					btScalar mass = nodeData.m_inverseMass? 1./nodeData.m_inverseMass : 0.f;
					psb->appendNode(position,mass);
					btSoftBody::Node* node = &psb->m_nodes[psb->m_nodes.size()-1];
					node->m_area = nodeData.m_area;
					node->m_battach = nodeData.m_attach;
					node->m_f.deSerializeFloat(nodeData.m_accumulatedForce);
					node->m_im = nodeData.m_inverseMass;

					btSoftBody::Material** matPtr = m_materialMap.find(nodeData.m_material);
					if (matPtr && *matPtr)
					{
						node->m_material = *matPtr;
					} else
					{
						printf("no mat?\n");
					}

					node->m_n.deSerializeFloat(nodeData.m_normal);
					node->m_q = node->m_x;
					node->m_v.deSerializeFloat(nodeData.m_velocity);

				}

				for (i=0;i<softBodyData->m_numLinks;i++)
				{
					SoftBodyLinkData& linkData = softBodyData->m_links[i];
					btSoftBody::Material** matPtr = m_materialMap.find(linkData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1],*matPtr);
					} else
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1]);
					}
					btSoftBody::Link* link = &psb->m_links[psb->m_links.size()-1];
					link->m_bbending = linkData.m_bbending;
					link->m_rl = linkData.m_restLength;
				}

				for (i=0;i<softBodyData->m_numFaces;i++)
				{
					SoftBodyFaceData& faceData = softBodyData->m_faces[i];
					btSoftBody::Material** matPtr = m_materialMap.find(faceData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2],*matPtr);
					} else
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2]);
					}
					btSoftBody::Face* face = &psb->m_faces[psb->m_faces.size()-1];
					face->m_normal.deSerializeFloat(faceData.m_normal);
					face->m_ra = faceData.m_restArea;
				}



				//anchors
				for (i=0;i<softBodyData->m_numAnchors;i++)
				{
					btCollisionObject** colAptr = m_bodyMap.find(softBodyData->m_anchors[i].m_rigidBody);
					if (colAptr && *colAptr)
					{
						btRigidBody* body = btRigidBody::upcast(*colAptr);
						if (body)
						{
							bool disableCollision = false;
							btVector3 localPivot;
							localPivot.deSerializeFloat(softBodyData->m_anchors[i].m_localFrame);
							psb->appendAnchor(softBodyData->m_anchors[i].m_nodeIndex,body,localPivot, disableCollision);
						}
					}
				}

				if (softBodyData->m_pose)
				{
					psb->m_pose.m_aqq.deSerializeFloat(  softBodyData->m_pose->m_aqq);
					psb->m_pose.m_bframe = (softBodyData->m_pose->m_bframe!=0);
					psb->m_pose.m_bvolume = (softBodyData->m_pose->m_bvolume!=0);
					psb->m_pose.m_com.deSerializeFloat(softBodyData->m_pose->m_com);

					psb->m_pose.m_pos.resize(softBodyData->m_pose->m_numPositions);
					for (i=0;i<softBodyData->m_pose->m_numPositions;i++)
					{
						psb->m_pose.m_pos[i].deSerializeFloat(softBodyData->m_pose->m_positions[i]);
					}
					psb->m_pose.m_rot.deSerializeFloat(softBodyData->m_pose->m_rot);
					psb->m_pose.m_scl.deSerializeFloat(softBodyData->m_pose->m_scale);
					psb->m_pose.m_wgh.resize(softBodyData->m_pose->m_numWeigts);
					for (i=0;i<softBodyData->m_pose->m_numWeigts;i++)
					{
						psb->m_pose.m_wgh[i] = softBodyData->m_pose->m_weights[i];
					}
					psb->m_pose.m_volume = softBodyData->m_pose->m_restVolume;
				}

#if 1
				psb->m_cfg.piterations=softBodyData->m_config.m_positionIterations;
				psb->m_cfg.diterations=softBodyData->m_config.m_driftIterations;
				psb->m_cfg.citerations=softBodyData->m_config.m_clusterIterations;
				psb->m_cfg.viterations=softBodyData->m_config.m_velocityIterations;

				//psb->setTotalMass(0.1);
				psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_)softBodyData->m_config.m_aeroModel;
				psb->m_cfg.kLF = softBodyData->m_config.m_lift;
				psb->m_cfg.kDG = softBodyData->m_config.m_drag;
				psb->m_cfg.kMT = softBodyData->m_config.m_poseMatch;
				psb->m_cfg.collisions = softBodyData->m_config.m_collisionFlags;
				psb->m_cfg.kDF = 1.f;//softBodyData->m_config.m_dynamicFriction;
				psb->m_cfg.kDP = softBodyData->m_config.m_damping;
				psb->m_cfg.kPR = softBodyData->m_config.m_pressure;
				psb->m_cfg.kVC = softBodyData->m_config.m_volume;
				psb->m_cfg.kAHR = softBodyData->m_config.m_anchorHardness;
				psb->m_cfg.kKHR = softBodyData->m_config.m_kineticContactHardness;
				psb->m_cfg.kSHR = softBodyData->m_config.m_softContactHardness;
				psb->m_cfg.kSRHR_CL = softBodyData->m_config.m_softRigidClusterHardness;
				psb->m_cfg.kSKHR_CL = softBodyData->m_config.m_softKineticClusterHardness;
				psb->m_cfg.kSSHR_CL = softBodyData->m_config.m_softSoftClusterHardness;
#endif

//				pm->m_kLST				=	1;

#if 1
				//clusters
				if (softBodyData->m_numClusters)
				{
					m_clusterBodyMap.insert(softBodyData->m_clusters,psb);
					int j;
					psb->m_clusters.resize(softBodyData->m_numClusters);
					for (i=0;i<softBodyData->m_numClusters;i++)
					{
						psb->m_clusters[i] = new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();
						psb->m_clusters[i]->m_adamping = softBodyData->m_clusters[i].m_adamping;
						psb->m_clusters[i]->m_av.deSerializeFloat(softBodyData->m_clusters[i].m_av);
						psb->m_clusters[i]->m_clusterIndex = softBodyData->m_clusters[i].m_clusterIndex;
						psb->m_clusters[i]->m_collide = (softBodyData->m_clusters[i].m_collide!=0);
						psb->m_clusters[i]->m_com.deSerializeFloat(softBodyData->m_clusters[i].m_com);
						psb->m_clusters[i]->m_containsAnchor = (softBodyData->m_clusters[i].m_containsAnchor!=0);
						psb->m_clusters[i]->m_dimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[0]);
						psb->m_clusters[i]->m_dimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[1]);

						psb->m_clusters[i]->m_framerefs.resize(softBodyData->m_clusters[i].m_numFrameRefs);
						for (j=0;j<softBodyData->m_clusters[i].m_numFrameRefs;j++)
						{
							psb->m_clusters[i]->m_framerefs[j].deSerializeFloat(softBodyData->m_clusters[i].m_framerefs[j]);
						}
						psb->m_clusters[i]->m_nodes.resize(softBodyData->m_clusters[i].m_numNodes);
						for (j=0;j<softBodyData->m_clusters[i].m_numNodes;j++)
						{
							int nodeIndex = softBodyData->m_clusters[i].m_nodeIndices[j];
							psb->m_clusters[i]->m_nodes[j] = &psb->m_nodes[nodeIndex];
						}

						psb->m_clusters[i]->m_masses.resize(softBodyData->m_clusters[i].m_numMasses);
						for (j=0;j<softBodyData->m_clusters[i].m_numMasses;j++)
						{
							psb->m_clusters[i]->m_masses[j] = softBodyData->m_clusters[i].m_masses[j];
						}
						psb->m_clusters[i]->m_framexform.deSerializeFloat(softBodyData->m_clusters[i].m_framexform);
						psb->m_clusters[i]->m_idmass = softBodyData->m_clusters[i].m_idmass;
						psb->m_clusters[i]->m_imass = softBodyData->m_clusters[i].m_imass;
						psb->m_clusters[i]->m_invwi.deSerializeFloat(softBodyData->m_clusters[i].m_invwi);
						psb->m_clusters[i]->m_ldamping = softBodyData->m_clusters[i].m_ldamping;
						psb->m_clusters[i]->m_locii.deSerializeFloat(softBodyData->m_clusters[i].m_locii);
						psb->m_clusters[i]->m_lv.deSerializeFloat(softBodyData->m_clusters[i].m_lv);
						psb->m_clusters[i]->m_matching = softBodyData->m_clusters[i].m_matching;
						psb->m_clusters[i]->m_maxSelfCollisionImpulse = 0;//softBodyData->m_clusters[i].m_maxSelfCollisionImpulse;
						psb->m_clusters[i]->m_ndamping = softBodyData->m_clusters[i].m_ndamping;
						psb->m_clusters[i]->m_ndimpulses = softBodyData->m_clusters[i].m_ndimpulses;
						psb->m_clusters[i]->m_nvimpulses = softBodyData->m_clusters[i].m_nvimpulses;
						psb->m_clusters[i]->m_selfCollisionImpulseFactor = softBodyData->m_clusters[i].m_selfCollisionImpulseFactor;
						psb->m_clusters[i]->m_vimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[0]);
						psb->m_clusters[i]->m_vimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[1]);

					}
					//psb->initializeClusters();
					//psb->updateClusters();

				}
#else

				psb->m_cfg.piterations	=	2;
				psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS;
				//psb->setTotalMass(50,true);
				//psb->generateClusters(64);
				//psb->m_cfg.kDF=1;
				psb->generateClusters(8);


#endif //



				psb->updateConstants();
				m_softRigidWorld->getWorldInfo().m_dispatcher = m_softRigidWorld->getDispatcher();

				m_softRigidWorld->addSoftBody(psb);


			}
		}


		//now the soft body joints
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				btSoftBody** sbp = m_softBodyMap.find(softBodyData);
				if (sbp && *sbp)
				{
					btSoftBody* sb = *sbp;
					for (int i=0;i<softBodyData->m_numJoints;i++)
					{
						btSoftBodyJointData* sbjoint = &softBodyData->m_joints[i];


						btSoftBody::Body bdyB;

						btSoftBody* sbB = 0;
						btTransform transA;
						transA.setIdentity();
						transA = sb->m_clusters[0]->m_framexform;

						btCollisionObject** colBptr = m_bodyMap.find(sbjoint->m_bodyB);
						if (colBptr && *colBptr)
						{
							btRigidBody* rbB = btRigidBody::upcast(*colBptr);
							if (rbB)
							{
								bdyB = rbB;
							} else
							{
								bdyB = *colBptr;
							}
						}


						btSoftBody** bodyBptr = m_clusterBodyMap.find(sbjoint->m_bodyB);
						if (bodyBptr && *bodyBptr )
						{
							sbB = *bodyBptr;
							bdyB = sbB->m_clusters[0];
						}


						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Linear)
						{
							btSoftBody::LJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.position = transA*relA;
							sb->appendLinearJoint(specs,sb->m_clusters[0],bdyB);
						}

						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Angular)
						{
							btSoftBody::AJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.axis = transA.getBasis()*relA;
							sb->appendAngularJoint(specs,sb->m_clusters[0],bdyB);
						}
					}
				}

			}
		}

		return result;

	}

