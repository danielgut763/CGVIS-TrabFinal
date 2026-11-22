// Stub de GetMeshLocalBBox para os testes sem janela: bboxes locais reais das
// malhas de data/*.obj (medidas dos vertices).
#include <string>
#include <glm/vec3.hpp>
bool GetMeshLocalBBox(const std::string& m, glm::vec3* a, glm::vec3* b)
{
    glm::vec3 lo(-0.5f), hi(0.5f);                       // cube, die, rubik, cone
    if (m == "the_pawn")   { lo = glm::vec3(-0.335f, -0.5f, -0.335f);   hi = -lo; }
    if (m == "the_bunny")  { lo = glm::vec3(-1.0f, -0.991f, -0.775f);   hi = -lo; }
    if (m == "the_sphere") { lo = glm::vec3(-1.0f);                     hi = glm::vec3(1.0f); }
    if (a) *a = lo;
    if (b) *b = hi;
    return true;
}
