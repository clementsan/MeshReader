#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
//#include <vtkCellArray.h>

#include <itkDefaultDynamicMeshTraits.h>
#include <itkMetaMeshConverter.h>
#include "itkMeshTovtkPolyData.h"

// ITK includes
#include <itkCovariantVector.h>
#include <itkVector.h>
#include <itkArray.h>

static const double PI = 3.14159265358979323846;
typedef itk::CovariantVector<double,3> CovariantVectorType;
typedef itk::Vector<int,3> TriangleType;
typedef itk::Vector<double,3> VectorType;


using namespace std;

int main (int argc, char * argv[])
{
  if (argc != 3)
    {
      cerr<<argv[0]<<" <InputMesh> <OutputMesh>"<<endl;
      return EXIT_FAILURE;
    }
  else
    {
      cout<<"Mesh Reader!"<<endl;
      vector<CovariantVectorType> m_vVertices;
      vector<TriangleType> m_vTriangles;
      
      // Reading mesh
      string infile = argv[1];
      int lastPoint = infile.rfind( '.' );
      string fileExtension;
      fileExtension = infile.substr( lastPoint );
      cout<<"fileExtension: "<<fileExtension<<endl;
      vtkPolyDataReader *m_vtkPolyDataReader;
      vtkPolyData *m_mesh;
      //vtkCellArray *m_Vertices;
      if (fileExtension.compare ( ".vtk" ) == 0 )
	{
	  cout<<"VTK file\n"<<endl;
	  /*m_vtkPolyDataReader = vtkPolyDataReader::New();
	    m_vtkPolyDataReader->SetFileName(argv[1]);
	    m_vtkPolyDataReader->Update();
	    m_mesh = m_vtkPolyDataReader->GetOutput();
	    
	    m_Vertices = m_mesh->GetVerts();*/
	}
      else if (fileExtension.compare ( ".meta" ) == 0 )
	{
	  cout<<"Meta file\n"<<endl;
	  /*typedef itk::DefaultDynamicMeshTraits < double, 3, 3, double, double > MeshTraitsType ;
	    typedef itk::Mesh < double, 3, MeshTraitsType > itkMeshType ;
	    typedef itk::MeshSpatialObject < itkMeshType > itkMeshSOType ;
	    typedef itk::MetaMeshConverter < 3, double, MeshTraitsType > MeshConverterType ;
	    
	    MeshConverterType *itkConverter = new MeshConverterType () ;
	    itkMeshSOType::Pointer meshSO = itkConverter->ReadMeta (argv[1]);
	    itkMeshType::Pointer mesh = meshSO->GetMesh () ;
	    delete ( itkConverter ) ;
	    
	    itkMeshTovtkPolyData * ITK_VTK_Converter = new itkMeshTovtkPolyData ;
	    ITK_VTK_Converter->SetInput ( mesh ) ;
	    m_mesh = ITK_VTK_Converter->GetOutput () ;
	    mesh->DisconnectPipeline () ;
	    delete ( ITK_VTK_Converter ) ;*/
	}
      
      int m_NbVertices;
      int m_NbTriangles;
	
      cout<<"\nReading Mesh..."<<endl;
      
      ifstream Infile;
      char Line[40], NbVertices[10], NbTriangles[10];
      string line;
      size_t found1, found2, length;
      TriangleType Triangle;
      CovariantVectorType Vertex1, Vertex2, Vertex3;
      int CurrentPoint, CurrentTriangle;
      
      m_vVertices.clear();  
      m_vTriangles.clear();  
      
      Infile.open(argv[1]);  
      
      while ( strncmp (Line, "POINTS", 6))
	Infile.getline (Line, 40);
      line = Line;
      found1 = line.find(' ');
      found2 = line.find(' ',found1+1);
      length = line.copy(NbVertices,found2-found1-1,found1+1);
      NbVertices[length]='\0';
      m_NbVertices = atoi(NbVertices);
      cout<<"NbVertices: "<<m_NbVertices<<endl;

      while ( strncmp (Line, "POINTS", 6))
	Infile.getline (Line, 40);
      for (int i = 0; i < m_NbVertices/3; i++ )
	{
	  Infile >> Vertex1[0] >> Vertex1[1] >> Vertex1[2] >> Vertex2[0] >> Vertex2[1] >> Vertex2[2] >> Vertex3[0] >> Vertex3[1] >> Vertex3[2];
	  //cout<<"Vertex1: "<<Vertex1[0]<<" "<<Vertex1[1]<<" "<<Vertex1[2]<<endl;
	  //cout<<"Vertex2: "<<Vertex2[0]<<" "<<Vertex2[1]<<" "<<Vertex2[2]<<endl;
	  //cout<<"Vertex3: "<<Vertex3[0]<<" "<<Vertex3[1]<<" "<<Vertex3[2]<<endl;
	  m_vVertices.push_back(Vertex1);
	  m_vVertices.push_back(Vertex2);
	  m_vVertices.push_back(Vertex3);	  
	}
      if ((m_NbVertices % 3) == 1)
	{
	  Infile >> Vertex1[0] >> Vertex1[1] >> Vertex1[2];
	  //cout<<"Vertex1: "<<Vertex1[0]<<" "<<Vertex1[1]<<" "<<Vertex1[2]<<endl;
	  m_vVertices.push_back(Vertex1);
	}
      else if ((m_NbVertices % 3) == 2)
	{
	  Infile >> Vertex1[0] >> Vertex1[1] >> Vertex1[2] >> Vertex2[0] >> Vertex2[1] >> Vertex2[2];
	  //cout<<"Vertex1: "<<Vertex1[0]<<" "<<Vertex1[1]<<" "<<Vertex1[2]<<endl;
	  //cout<<"Vertex2: "<<Vertex2[0]<<" "<<Vertex2[1]<<" "<<Vertex2[2]<<endl;
	  m_vVertices.push_back(Vertex1);
	  m_vVertices.push_back(Vertex2);
	}
      
      while ( strncmp (Line, "POLYGONS", 8))
	Infile.getline (Line, 40);
      line.clear();
      line = Line;
      found1 = line.find(' ');
      found2 = line.find(' ',found1+1);
      length = line.copy(NbTriangles,found2-found1-1,found1+1);
      NbTriangles[length]='\0';
      m_NbTriangles = atoi(NbTriangles);
      cout<<"NbTriangles: "<<m_NbTriangles<<endl;      
      
      //read the triangles in the file and set them as triangles
      for (int i = 0; i < m_NbTriangles; i++ )
	{
	  Infile >> CurrentTriangle >> Triangle[0] >> Triangle[1] >> Triangle[2];
	  //cout<<"Triangle: "<<Triangle[0]<<" "<<Triangle[1]<<" "<<Triangle[2]<<endl;
	  m_vTriangles.push_back(Triangle);
	}
      //close file
      Infile.close();      

      // WriteMesh
      string outfile = argv[2];
      lastPoint = outfile.rfind( '.' );
      fileExtension.clear();
      fileExtension = outfile.substr( lastPoint );
      cout<<"fileExtension: "<<fileExtension<<endl;
      
      if (fileExtension.compare ( ".vtk" ) == 0 )
	{
	  cout<<"VTK file\n"<<endl;
	  ofstream outfile ;
	  outfile.open(argv[2]);
	  // output header
	  outfile<<"# vtk DataFile Version 3.0"<<endl<<"vtk output"<<endl<<"ASCII"<<endl<<"DATASET POLYDATA"<<endl;
	  outfile<<"POINTS "<<m_NbVertices<<" float"<<endl;
	  for (int i = 0; i < m_NbVertices-3; i+=3 )
	    outfile<<m_vVertices[i][0]<<" "<<m_vVertices[i][1]<<" "<<m_vVertices[i][2]<<" "<<m_vVertices[i+1][0]<<" "<<m_vVertices[i+1][1]<<" "<<m_vVertices[i+1][2] \
		   <<" "<<m_vVertices[i+2][0]<<" "<<m_vVertices[i+2][1]<<" "<<m_vVertices[i+2][2]<<" "<<endl;
	  if ((m_NbVertices % 3) == 1)
	    outfile<<m_vVertices[m_NbVertices-1][0]<<" "<<m_vVertices[m_NbVertices-1][1]<<" "<<m_vVertices[m_NbVertices-1][2]<<" "<<endl;
	  else if ((m_NbVertices % 3) == 2)
	    outfile<<m_vVertices[m_NbVertices-2][0]<<" "<<m_vVertices[m_NbVertices-2][1]<<" "<<m_vVertices[m_NbVertices-2][2]<<" "<<m_vVertices[m_NbVertices-1][0]<<" "<<m_vVertices[m_NbVertices-1][1]<<" "<<m_vVertices[m_NbVertices-1][2]<<" "<<endl;
	  outfile<<"POLYGONS "<<m_NbTriangles<<" "<<4*m_NbTriangles<<endl;
	  for (int i = 0 ; i < m_NbTriangles ; i++)
	    outfile<<"3 "<<m_vTriangles[i][0]<<" "<<m_vTriangles[i][1]<<" "<<m_vTriangles[i][2]<<" "<<endl;
	  outfile<<endl;
	}
      else if (fileExtension.compare ( ".meta" ) == 0 )
	{
	  cout<<"Meta file\n"<<endl;
	  ofstream outfile ;
	  int i ;
  
	  outfile.open(argv[2]);
	  
	  // output header
	  outfile<<"ObjectType = Mesh"<<endl<<"NDims = 3"<<endl<<"ID = 0"<<endl ;
	  outfile<<"TransformMatrix = 1 0 0 0 1 0 0 0 1"<<endl<<"Offset = 0 0 0"<<endl<<"CenterOfRotation = 0 0 0"<<endl ;
	  outfile<<"ElementSpacing = 1 1 1"<<endl<<"PointType = MET_FLOAT"<<endl<<"PointDataType = MET_FLOAT"<<endl ;
	  outfile<<"CellDataType = MET_FLOAT"<<endl<<"NCellTypes = 1"<<endl<<"PointDim = ID x y ..."<<endl ;
	  outfile<<"NPoints = "<<m_NbVertices<<endl ;
	  
	  outfile<<"Points = "<<endl ;
	  
	  for ( i = 0 ; i < m_NbVertices ; i++ )
	    outfile<<i<<" "<<m_vVertices[i][0]<<" "<<m_vVertices[i][1]<<" "<<m_vVertices[i][2]<<endl;
	  
	  outfile<<endl<<"CellType = TRI"<<endl<<"NCells = "<<m_NbTriangles<<endl<<"Cells = "<<endl ;
	  
	  for ( i = 0 ; i < m_NbTriangles ; i++)
	    outfile<<i<<" "<<m_vTriangles[i][0]<<" "<<m_vTriangles[i][1]<<" "<<m_vTriangles[i][2]<<endl;
	  
	  outfile.close () ;
	}
      
      
    }
  return EXIT_SUCCESS;
}
