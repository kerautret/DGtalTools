// file Image2D.hpp
#ifndef _IMAGE2D_HPP_
#define _IMAGE2D_HPP_
#include <vector>

namespace rt {

/// Classe générique pour représenter des images 2D.
template <typename TValue>
class Image2D {
public:
  typedef Image2D<TValue>    Self;      // le type de *this
  typedef TValue             Value;     // le type pour la valeur des pixels
  typedef std::vector<Value> Container; // le type pour stocker les valeurs des pixels de l'image.
  typedef typename Container::iterator ContainerIterator;
  typedef typename Container::const_iterator ContainerConstIterator;
  /// Un itérateur (non-constant) simple sur l'image.
  struct Iterator : public ContainerIterator {
    Iterator( const ContainerIterator & other ) 
      : ContainerIterator( other )
    {}
    Iterator( Self & image, int x, int y ) 
      : ContainerIterator( image.m_data.begin() + image.index( x, y ) )
    {}
    Iterator& operator=( const ContainerIterator & other ) 
    {
      ContainerIterator::operator=( other );
      return *this;
    }

    // operator ContainerIterator() const
    // {
    //   return static_cast<ContainerIterator>(*this);
    // }
      
  };

  struct ConstIterator : public ContainerConstIterator {
    ConstIterator( const ContainerConstIterator & other ) 
      : ContainerConstIterator( other )
    {}
    ConstIterator( const Self & image, int x, int y ) 
      : ContainerConstIterator( image.m_data.begin() + image.index( x, y ) )
    {}
    ConstIterator& operator=( const ContainerConstIterator & other ) 
    {
      ContainerConstIterator::operator=( other );
      return *this;
    }

    // operator ContainerConstIterator() const
    // {
    //   return static_cast<ContainerConstIterator>(*this);
    // }
      
  };

  template <typename TAccessor> 
  struct GenericConstIterator : public Container::const_iterator {
    typedef TAccessor Accessor;
    typedef typename Accessor::Argument  ImageValue; // Color ou unsigned char
    typedef typename Accessor::Value     Value;      // unsigned char (pour ColorGreenAccessor)
    typedef typename Accessor::Reference Reference;  // ColorGreenReference (pour ColorGreenAccessor)
    
    GenericConstIterator( const Image2D<ImageValue>& image, int x, int y )
      : Container::const_iterator( image.m_data.begin() + image.index( x, y ) ) {}
    
    // Accès en lecture (rvalue)
    Value operator*() const
    { return Accessor::access( Container::const_iterator::operator*() ); }
    
  };

  template <typename TAccessor> 
  struct GenericIterator : public Container::iterator {
    typedef TAccessor Accessor;
    typedef typename Accessor::Argument  ImageValue; // Color ou unsigned char
    typedef typename Accessor::Value     Value;      // unsigned char (pour ColorGreenAccessor)
    typedef typename Accessor::Reference Reference;  // ColorGreenReference (pour ColorGreenAccessor)
    
    GenericIterator( Image2D<ImageValue>& image, int x, int y )
      : Container::iterator( image.m_data.begin() + image.index( x, y ) ) {}
    
    // Accès en lecture (rvalue)
    Value operator*() const
    { return Accessor::access( Container::iterator::operator*() ); }

    // Accès en lecture (rvalue)
    Reference operator*()
    { return Accessor::access( Container::iterator::operator*() ); }
    
  };

  // Constructeur par défaut
  Image2D();
  // Constructeur avec taille w x h. Remplit tout avec la valeur g
  // (par défaut celle donnée par le constructeur par défaut).
  Image2D( int w, int h, Value g = Value() );
  
  // Remplit l'image avec la valeur \a g.
  void fill( Value g );
  
  /// @return la largeur de l'image.
  int w() const;
  /// @return la hauteur de l'image.
  int h() const;

  /// @return un itérateur pointant sur le début de l'image
  Iterator begin() { return start( 0, 0 ); }
  /// @return un itérateur pointant après la fin de l'image
  Iterator end()   { return start( 0, h() ); }
  /// @return un itérateur pointant sur le pixel (x,y).
  Iterator start( int x, int y ) { return Iterator( *this, x, y ); }

  template <typename Accessor>
  GenericConstIterator< Accessor > start( int x = 0, int y = 0 ) const
  { return GenericConstIterator< Accessor >( *this, x, y ); }

  template <typename Accessor>
  GenericConstIterator< Accessor > begin() const
  { return start< Accessor >( 0, 0 ); }

  template <typename Accessor>
  GenericConstIterator< Accessor > end() const
  { return start< Accessor >( 0, h() ); }

  template <typename Accessor>
  GenericIterator< Accessor > start( int x = 0, int y = 0 )
  { return GenericIterator< Accessor >( *this, x, y ); }

  template <typename Accessor>
  GenericIterator< Accessor > begin()
  { return start< Accessor >( 0, 0 ); }

  template <typename Accessor>
  GenericIterator< Accessor > end()
  { return start< Accessor >( 0, h() ); }
   
  /// Accesseur read-only à la valeur d'un pixel.
  /// @return la valeur du pixel(i,j)
  Value at( int i, int j ) const;
  
  /// Accesseur read-write à la valeur d'un pixel.
  /// @return une référence à la valeur du pixel(i,j)
  Value& at( int i, int j );
  
private:
  Container m_data; // mes données; évitera de faire les allocations dynamiques
  int m_width; // ma largeur
  int m_height; // ma hauteur
  
  /// @return l'index du pixel (x,y) dans le tableau \red m_data.
  int index( int i, int j ) const;
};

template <typename TValue>
Image2D<TValue>::Image2D()
  : m_data(), m_width( 0 ), m_height( 0 )
{}

template <typename TValue>
Image2D<TValue>::Image2D( int w, int h, Value g )
  : m_data( w*h, g ), m_width( w ), m_height( h )
{}

template <typename TValue>
void
Image2D<TValue>::fill( Value g )
{
  m_data.fill( g );
}

template <typename TValue>
int
Image2D<TValue>::w() const
{ return m_width; }

template <typename TValue>
int
Image2D<TValue>::h() const
{ return m_height; }

template <typename TValue>
typename Image2D<TValue>::Value
Image2D<TValue>::at( int i, int j ) const
{
  return m_data[ index( i, j ) ];
}

template <typename TValue>
typename Image2D<TValue>::Value&
Image2D<TValue>::at( int i, int j )
{
  return m_data[ index( i, j ) ];
}

template <typename TValue>
int
Image2D<TValue>::index( int i, int j ) const
{
  return i + j *w();
}

} // namespace rt
#endif // _IMAGE2D_HPP_
