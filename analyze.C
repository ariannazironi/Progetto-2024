#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "TCanvas.h"
#include "TGraph.h"

void analyze() {
  std::ifstream file("simulation_data.txt");

  if (!file.is_open()) {
    std::cerr << "Errore nell'aprire il file" << std::endl;
    return;
  }

  float time;
  float mean_dist;
  float dev_dist;
  float mean_speed;
  float dev_speed;

  std::vector<float> times;
  std::vector<float> mean_dists;
  std::vector<float> mean_speeds;

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::istringstream stream(line);

    if (stream >> time >> mean_dist >> dev_dist >> mean_speed >> dev_speed) {
      times.push_back(time);
      mean_speeds.push_back(mean_speed);
      mean_dists.push_back(mean_dist);
    } else {
      std::cerr << "Errore nella lettura dei dati dalla riga: " << line
                << std::endl;
    }
  }

  if (times.empty()) {
    std::cerr << "Nessun dato valido trovato nel file" << std::endl;
    return;
  }

  TGraph *graph_speed =
      new TGraph(times.size(), times.data(), mean_speeds.data());
  graph_speed->SetTitle("Media della velocità vs Tempo");
  graph_speed->GetXaxis()->SetTitle("Tempo (s)");
  graph_speed->GetYaxis()->SetTitle("Velocità Media");
  graph_speed->SetMarkerStyle(20);
  graph_speed->SetMarkerColor(kBlue);

  TGraph *graph_distance =
      new TGraph(times.size(), times.data(), mean_dists.data());
  graph_distance->SetTitle("Media della distanza vs Tempo");
  graph_distance->GetXaxis()->SetTitle("Tempo (s)");
  graph_distance->GetYaxis()->SetTitle("Distanza Media");
  graph_distance->SetMarkerStyle(21);
  graph_distance->SetMarkerColor(kRed);

  TCanvas *c1 = new TCanvas("c1", "Mean Distance vs Time", 800, 600);
  graph_distance->Draw("APL");

  TCanvas *c2 = new TCanvas("c2", "Mean Speed vs Time", 800, 600);
  graph_speed->Draw("APL");

  c1->Update();
  c2->Update();
}
