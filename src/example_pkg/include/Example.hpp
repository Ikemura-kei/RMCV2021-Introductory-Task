#include <string>

namespace Example
{
class IKEMURA
{
 public:
   IKEMURA() = default;

   IKEMURA(const std::string &word);

   void sayHi();

   void sayBye();

   void sayWord();

 private:
   std::string word = "CV Task";
};

}  // namespace Example
