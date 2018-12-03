#include "map_evaluator.h"
#include "semantic_mapper/object.cpp"

void MapEvaluator::setReference(const std::string & filename){
  if(filename.empty())
    return;

  _reference.clear();

  YAML::Node map = YAML::LoadFile(filename);

  for(YAML::const_iterator it=map.begin(); it!=map.end(); ++it){
    const std::string &key=it->first.as<std::string>();
    const GtObject &value=it->second.as<GtObject>();

    _reference.insert(std::make_pair(key,value));
  }
}

void MapEvaluator::setCurrent(const ObjectPtrVector * current){
  if(!current)
    return;

  _current.clear();

  for(size_t i=0; i<current->size(); ++i){
    const std::string &key = current->at(i)->model();
    const Object &value = *(current->at(i));

    _current.insert(std::make_pair(key,value));
  }
}

void MapEvaluator::compute(){

  std::ofstream file;
  file.open("evaluation.txt");

  for(GtObjectStringMap::iterator it = _reference.begin(); it != _reference.end(); ++it){
    const std::string &reference_model = it->first;
    const GtObject &reference_object = it->second;

    ObjectStringMap::iterator jt = _current.find(reference_model);
    if(jt!=_current.end()){
      //model found
      std::cerr << reference_model << ": found!" << std::endl;
      file << reference_model << ": found!" << std::endl;
      const Object &current_object = jt->second;

      //evaluate object position
      const Eigen::Vector3f &ref_pos = reference_object.position();
      const Eigen::Vector3f &cur_pos = current_object.position();
      float pos_error = (ref_pos - cur_pos).norm();
      std::cerr << "\t>>position error: " << pos_error << std::endl;
      file << "\t>>position error: " << pos_error << std::endl;

      //evaluate object volume
      const Eigen::Vector3f &ref_min = reference_object.min();
      const Eigen::Vector3f &ref_max = reference_object.max();
      const Eigen::Vector3f &cur_min = current_object.min();
      const Eigen::Vector3f &cur_max = current_object.max();
      float ref_volume = (ref_max.x() - ref_min.x())*(ref_max.y() - ref_min.y())*(ref_max.z() - ref_min.z());
      float cur_volume = (cur_max.x() - cur_min.x())*(cur_max.y() - cur_min.y())*(cur_max.z() - cur_min.z());
      float vol_error = std::fabs(ref_volume-cur_volume);
      std::cerr << "\t>>volume error: " << vol_error << std::endl;
      file << "\t>>volume error: " << vol_error << std::endl;
    }
  }

  file.close();
}

void MapEvaluator::storeMap(const ObjectPtrVector *current){
  if(!current)
    return;

  YAML::Node node(YAML::NodeType::Sequence);
  for(size_t i=0; i<current->size(); ++i){
    const std::string &key = current->at(i)->model();
    const Object &obj = *(current->at(i));
    node[key]=obj;
  }

  std::ofstream fout("semantic_map.yaml");
  fout << node;
}
