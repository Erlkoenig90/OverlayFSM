#ifndef OVERLAY_FSM_HH_
#define OVERLAY_FSM_HH_

#include <new>
#include <utility>
#include <type_traits>

#if defined(__GNUC__) && !defined(overlayfsm_always_inline)
#	define overlayfsm_always_inline __attribute__((always_inline)) inline
#endif

namespace OverlayFSM {
	namespace Helper {
		/// Hilfsklasse für Tag-Parameter.
		template <typename T>
		struct UnionTag {};
		
	#if defined(__GNUC__) && !defined(__TASKING__)
		/**
		 * Da GCC & Clang fälschlicherweise erfordern, dass alle nicht-statischen Member einer union literal types sein müssen damit die union selbst ein literal type ist,
		 * werden die Member in ein "Uninitialized" verpackt. Für literal types ist dies ein trivialer Wrapper, für nicht-literal types enthält Uninitialized statt des Types
		 * flachen Speicher, der per placement-new als Instanz der Klasse genutzt wird. Dadurch ist Uninitialized immer ein literal type.
		 *
		 * Auf literal types kann nicht (mehr) sicher mit std::is_literal geprüft werden, verwende daher is_trivially_destructible, denn GCC & Clang stolpern ohnehin über den Destruktor.
		 */

		template<typename Type, bool = std::is_trivially_destructible<Type>::value>
		struct Uninitialized;
		
		/// Spezialisierung für literal types
		template<typename Type>
		struct Uninitialized<Type, true> {
			/// Dieser Konstruktor gibt einfach nur die Argumente weiter. Da "Type" literal ist, ist Uninitalized auch literal und bietet einen constexpr-Konstruktor.
			template<typename... Args>
			constexpr Uninitialized (Args&&... args) : storage(std::forward<Args>(args)...) { }

			constexpr Type& get () {
                // Hack: In C++11 ist der Typ "const" wegen constexpr
			    return const_cast<Type&> (storage);
			}

			Type storage;
		};

		/// Spezialisierung für nicht-literal types
		template<typename Type>
		struct Uninitialized<Type, false> {
			/// Dieser Konstruktor erstellt die Instanz per Placement New und kann daher normalerweise nicht als constexpr genutzt werden. Uninitialized gilt dennoch als literal type.
			template<typename... Args>
	#if __cplusplus >= 201402L
			constexpr
	#endif
			Uninitialized (Args&&... args) {
				::new (&storage) Type (std::forward<Args>(args)...);
			}

            constexpr Type& get () {
                // Hack: In C++11 ist der Typ "const" wegen constexpr
                return reinterpret_cast<Type&> (const_cast<typename std::remove_const<decltype (storage)>::type&> (storage));
            }

			// Speicher für den gewünschten Typ anfordern
			typename std::aligned_storage <sizeof(Type), std::alignment_of<Type>::value>::type storage;
		};
	#else
		/// Für konforme Compiler keine Sonderbehandlung nötig
		template<typename Type>
		struct Uninitialized {
			/// Dieser Konstruktor gibt einfach nur die Argumente weiter. Da "Type" literal ist, ist Uninitalized auch literal und bietet einen constexpr-Konstruktor.
			template<typename... Args>
			constexpr Uninitialized (Args&&... args) : storage(std::forward<Args>(args)...) { }

            constexpr Type& get () {
                // Hack: In C++11 ist der Typ "const" wegen constexpr
                return const_cast<Type&> (storage);
            }

			Type storage;
		};
	#endif
		
		/// Dies ist eine variadische rekursive Union, die für jeden übergebenen Typ eine Member-Variable hat. Solange einer der Typen ein literal type ist, ist auch die union einer und kann per constexpr erstellt werden.
		template <typename... Types>
		union MultiUnion;
		
		/// Spezialisierung für leere Union
		template <>
		union MultiUnion<> { };
		
		template <typename First, typename... Rest>
		union MultiUnion<First, Rest...> {
			/// Der default constructor legt keine Instanz an
			constexpr MultiUnion () : rest {} {}
			
			/// Diesem Konstruktor wird per tag-Parameter die Klasse, die erstellt weden soll, übergeben.
			template <typename... Args>
			constexpr MultiUnion (UnionTag<First>, Args&&... args) : first { std::forward<Args> (args) ... } {}
			
			/// Diesem Konstruktor wird per tag-Parameter die Klasse, die erstellt weden soll, übergeben.
			template <typename Type, typename... Args>
			constexpr MultiUnion (UnionTag<Type> tag, Args&&... args) : rest { tag, std::forward<Args> (args)... } {}
			
			/// Erstellt eine Instanz des angegebenen Types
			template <typename... Args>
			First* construct (UnionTag<First>, Args&&... args) {
				return new (&first.storage) First (std::forward<Args> (args)...);
			}
			
			/// Erstellt eine Instanz des angegebenen Types
			template <typename Type, typename... Args>
			Type* construct (UnionTag<Type> tag, Args&&... args) {
				return rest.construct (tag, std::forward<Args> (args)...);
			}
			
			/// Liefert eine Referenz auf den angegeben Typ (falls dieser der aktive ist)
			constexpr First& get (UnionTag<First>) {
				return first.get ();
			}
			
			/// Liefert eine Referenz auf den angegeben Typ (falls dieser der aktive ist)
			template <typename Type>
			constexpr Type& get (UnionTag<Type> tag) {
				return rest.get (tag);
			}
			
			/// Instanz des Typs
			Uninitialized<First> first;

			/// Rekursionsschritt auf nächste Teil-Union
			MultiUnion<Rest...> rest;
		};

	#if __cplusplus >= 201703L
		template <bool... V>
		struct And {
			static constexpr bool value = (V && ...);
		};
		template <bool... V>
		struct Or {
			static constexpr bool value = (V || ...);
		};
	#else
		template <bool... V>
		struct And {
			static constexpr bool value = std::is_same<And, And<(static_cast<void> (V), true)...>>::value;
		};
		template <bool... V>
		struct Or {
			static constexpr bool value = !And<!V...>::value;
		};
	#endif

		/**
		 * Mixin für OverlayFSM; ruft this->destroy() NUR auf, wenn TriviallyDestructible=false ist.
		 * Damit wird der Destruktor des aktuellen Zustands aufgerufen, wenn die OverlayFSM-Instanz gelöscht wird und der Destruktor nicht-trivial ist;
		 * ist der Destruktor trivial/nicht vorhanden, hat MixinDestructor und damit auch OverlayFSM keinen Destruktur und ist damit auch trivially destructible.
		 */
		template <typename Top, bool TriviallyDestructible>
		class MixinDestructor;

		template <typename Top>
		class MixinDestructor<Top, false> {
			public:
				~MixinDestructor () { static_cast<Top*> (this)->destroy (); }
		};

		template <typename Top>
		class MixinDestructor<Top, true> {};
	}

	/**
	 * Implementierung des State Pattern, bei dem die Instanzen der einzelnen Zustands-Klassen im selben Speicher gehalten werden,
	 * um Speicher zu sparen.
	 * Beim Verlassen eines Zustands, oder wenn der Destruktor von OverlayFSM aufgerufen wird, wird der Destruktor des aktuellen Zustands
	 * aufgerufen. Dies geschieht aber über die Basisklasse. Ist der Destruktor der Basisklasse als virtual markiert, wird somit der Destruktor
	 * der abgeleiteten, tatsächlichen Zustandsklasse aufgerufen; falls nicht nur der Destruktor der Basisklasse aufgerufen.
	 *
	 * @param StateBase_	Basisklasse der Zustandsklassen
	 * @param States		Die Zustandsklassen, müssen von StateBase_ ableiten
	 */
	template <typename StateBase_, typename... States>
	class OverlayFSM : public Helper::MixinDestructor<OverlayFSM<StateBase_, States...>,
#ifdef __TASKING__
	false> { friend class Helper::MixinDestructor<OverlayFSM<StateBase_, States...>, false>;
#else
	std::is_trivially_destructible<StateBase_>::value
	> {
		friend class Helper::MixinDestructor<OverlayFSM<StateBase_, States...>, std::is_trivially_destructible<StateBase_>::value>;
#endif
		public:
			using StateBase = StateBase_;
			
			// Konsistenzprüfungen der Parameter
			static_assert (sizeof...(States) > 0, "OverlayFSM must have at least one state!");
#ifndef __TASKING__
			static_assert (Helper::And<std::is_base_of<StateBase, States>::value...>::value, "All states must be derived from the StateBase class!");
			static_assert (Helper::And<!std::is_abstract<States>::value...>::value, "State classes may not contain pure virtual member functions!");
#endif
		private:
			Helper::MultiUnion<States...> m_memory;
			// Zeiger auf den aktuellen Zustand; zeigt in m_stateMem hinein.
			StateBase* m_current;

			/// Ruft den Destruktor des aktuellen Zustands, auf dem Typ der Basisklasse, auf, falls er nicht-trivial ist. Tut nichts falls der Destruktor trivial ist (SFINAE).
#ifndef __TASKING__
			template <typename X = StateBase, class = typename std::enable_if<!std::is_trivially_destructible<X>::value>::type>
#endif
			void destroy () {
				current ().~StateBase ();
			}

#ifndef __TASKING__
			template <typename X = StateBase, class = typename std::enable_if<std::is_trivially_destructible<X>::value>::type>
			void destroy (...) {
			}
#endif
		public:
			/**
			 * Initialisiert die FSM mit dem gewünschten Zustand. Der erste Parameter kann einen beliebigen Wert (zB nullptr) haben,
			 * muss aber vom Typ des gewünschten Initialzustands sein (Tag-Parameter).
			 *
			 * @param S		Typ des gewünschten Initialzustands
			 * @param args	Parameter, die an den Konstruktor des Initialzustands übergeben werden.
			 */
			template <typename S, typename... Args>
			overlayfsm_always_inline constexpr OverlayFSM (S*, Args&&... args) : m_memory {Helper::UnionTag<S> {}, std::forward<Args> (args)...}, m_current { &m_memory.get (Helper::UnionTag<S> {}) } {  }

			/// Gibt eine Referenz auf den aktuellen Zustand zurück, als Typ der Basisklasse.
			overlayfsm_always_inline StateBase& current () { return *m_current; }
			/// Gibt eine konstante Referenz auf den aktuellen Zustand zurück, als Typ der Basisklasse.
			overlayfsm_always_inline const StateBase& current () const { return *m_current; }

			/**
			 * Wechselt in einen neuen Zustand über. Ruft den Destruktor des aktuellen Zustands, auf dem Typ der Basisklasse, auf.
			 *
			 * @param S		Typ des neuen Zustands
			 * @param args	An den Konstruktor des neuen Zustands zu übergebende Parameter.
			 * @return		Die Instanz des neuen Zustands
			 */
			template <typename S, typename... Args>
			overlayfsm_always_inline S& go (Args&&... args) {
				// Prüfe, ob der Zustand auch in den Klassem-Template-Parametern (States...) enthalten ist
				static_assert (Helper::Or<std::is_same<States, S>::value...>::value, "Nonexisting state passed to OverlayFSM::go");
				
				// Destruktor aufrufen
				destroy ();
				
				S& s = *m_memory.construct (Helper::UnionTag<S> {}, std::forward<Args> (args)...);
				m_current = &s;

				return s;
			}
	};
}

#endif
